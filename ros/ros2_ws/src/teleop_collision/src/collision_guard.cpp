#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include "clearance_policy.hpp"

class CollisionGuard
{
public:
  explicit CollisionGuard(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    group_ = node_->declare_parameter<std::string>("move_group_name", "manipulator");
    const auto joint_topic = node_->declare_parameter<std::string>("joint_topic", "/joint_states");
    frame_ = node_->declare_parameter<std::string>("floor.frame", "base_link");
    floor_id_ = node_->declare_parameter<std::string>("floor.id", "teleop_floor");
    const auto floor_z = node_->declare_parameter<double>("floor.top_z", -0.01);
    const auto size = node_->declare_parameter<std::vector<double>>("floor.size", {4., 4., 2.});
    const auto xy = node_->declare_parameter<std::vector<double>>("floor.center_xy", {0., 0.});
    const auto exempt = node_->declare_parameter<std::vector<std::string>>(
        "floor.mount_links", {"base_link", "base_link_inertia"});
    feedback_timeout_ = node_->declare_parameter<double>("feedback_timeout", 0.25);
    max_lead_ = node_->declare_parameter<double>("max_joint_lead", 0.15);
    path_step_ = node_->declare_parameter<double>("path_step", 0.02);
    scene_margin_ = node_->declare_parameter<double>("scene_margin", 0.005);
    self_margin_ = node_->declare_parameter<double>("self_margin", 0.003);
    for (const auto value : {feedback_timeout_, max_lead_, path_step_, scene_margin_, self_margin_})
      if (!std::isfinite(value) || value <= 0.) throw std::runtime_error("Invalid collision guard limit");
    if (size.size() != 3 || xy.size() != 2 || !std::isfinite(floor_z) ||
        std::any_of(size.begin(), size.end(), [](double v) { return !std::isfinite(v) || v <= 0.; }) ||
        std::any_of(xy.begin(), xy.end(), [](double v) { return !std::isfinite(v); }))
      throw std::runtime_error("Invalid floor dimensions");

    monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", "teleop_collision_scene");
    if (!monitor_->getPlanningScene()) throw std::runtime_error("Cannot load collision robot model");
    const auto model = monitor_->getRobotModel();
    joint_group_ = model->getJointModelGroup(group_);
    if (!joint_group_ || !model->hasLinkModel(frame_)) throw std::runtime_error("Invalid collision group/frame");
    // Only static mounting links may touch the floor; never exempt an arm link.
    for (const auto& link : exempt)
    {
      const auto* current = model->getLinkModel(link);
      if (!current) throw std::runtime_error("Unknown floor mounting link: " + link);
      while (current && current->getParentJointModel())
      {
        if (current->getParentJointModel()->getVariableCount() != 0)
          throw std::runtime_error("A moving link cannot be exempted from floor collisions: " + link);
        current = current->getParentLinkModel();
      }
    }

    floor_scene_.is_diff = true;
    auto& object = floor_scene_.world.collision_objects.emplace_back();
    object.id = floor_id_;
    object.header.frame_id = frame_;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.assign(size.begin(), size.end());
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.;
    pose.position.x = xy[0]; pose.position.y = xy[1]; pose.position.z = floor_z - size[2] / 2.;
    object.primitives.push_back(box);
    object.primitive_poses.push_back(pose);
    {
      planning_scene_monitor::LockedPlanningSceneRW scene(monitor_);
      if (!scene->processCollisionObjectMsg(object)) throw std::runtime_error("Cannot add floor");
      auto& acm = scene->getAllowedCollisionMatrixNonConst();
      for (const auto& link : exempt) acm.setEntry(floor_id_, link, true);
      acm.getMessage(floor_scene_.allowed_collision_matrix);
    }
    monitor_->startStateMonitor(joint_topic);
    monitor_->startSceneMonitor("/planning_scene");
    scene_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/teleop/collision_status", rclcpp::QoS(1).reliable().transient_local());
    service_ = node_->create_service<moveit_msgs::srv::GetStateValidity>(
        "/teleop/check_state_validity",
        [this](const std::shared_ptr<moveit_msgs::srv::GetStateValidity::Request> request,
               std::shared_ptr<moveit_msgs::srv::GetStateValidity::Response> response) { check(*request, *response); });
    timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
      // Reassert the configured floor for Servo, including after its restart.
      scene_pub_->publish(floor_scene_);
    });
    scene_pub_->publish(floor_scene_);
    status("WAITING_FOR_JOINT_STATES");
    RCLCPP_INFO(node_->get_logger(), "Floor top Z=%.3fm in %s; validating self/scene collisions and command paths",
                floor_z, frame_.c_str());
  }

private:
  void status(const std::string& text)
  {
    if (text == last_status_) return;
    last_status_ = text;
    std_msgs::msg::String message; message.data = text; status_pub_->publish(message);
    if (text.rfind("BLOCKED", 0) == 0) RCLCPP_WARN(node_->get_logger(), "%s", text.c_str());
  }

  bool measureClearance(const planning_scene::PlanningSceneConstPtr& scene, moveit::core::RobotState& state,
                        moveit_msgs::srv::GetStateValidity::Response& response,
                        ClearancePolicy::Distances& distances)
  {
    collision_detection::CollisionRequest request;
    request.group_name = group_;
    request.distance = true; request.contacts = true;
    request.max_contacts = 1; request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    const auto& acm = scene->getAllowedCollisionMatrix();
    for (const bool self : {false, true})
    {
      result.clear();
      if (self) scene->getCollisionEnvUnpadded()->checkSelfCollision(request, result, state, acm);
      else scene->getCollisionEnv()->checkRobotCollision(request, result, state, acm);
      distances[self ? 1 : 0] = result.distance;
      if (result.collision || !std::isfinite(result.distance) || result.distance <= 0.)
      {
        std::string reason = self ? "BLOCKED_SELF" : "BLOCKED_SCENE";
        for (const auto& pair : result.contacts)
        {
          moveit_msgs::msg::ContactInformation contact;
          contact.header.frame_id = scene->getPlanningFrame();
          contact.contact_body_1 = pair.first.first;
          contact.contact_body_2 = pair.first.second;
          response.contacts.push_back(contact);
          reason += ": " + pair.first.first + " / " + pair.first.second;
        }
        if (!result.collision) reason += ": invalid clearance=" + std::to_string(result.distance);
        status(reason);
        return false;
      }
    }
    return true;
  }

  void check(const moveit_msgs::srv::GetStateValidity::Request& request,
             moveit_msgs::srv::GetStateValidity::Response& response)
  {
    response.valid = false;
    const auto state_monitor = monitor_->getStateMonitor();
    const auto age = (node_->now() - state_monitor->getCurrentStateTime()).seconds();
    if (!state_monitor->haveCompleteState() || age < 0. || age > feedback_timeout_)
    { status("WAITING_FOR_FRESH_JOINT_STATES"); return; }
    const auto& input = request.robot_state.joint_state;
    const auto& names = joint_group_->getVariableNames();
    if (request.group_name != group_ || input.name.size() != names.size() || input.position.size() != names.size() ||
        std::set<std::string>(input.name.begin(), input.name.end()).size() != names.size())
    { status("INVALID_COMMAND_JOINTS"); return; }
    std::vector<double> target;
    for (const auto& name : names)
    {
      const auto found = std::find(input.name.begin(), input.name.end(), name);
      if (found == input.name.end()) { status("INVALID_COMMAND_JOINTS"); return; }
      const auto value = input.position[std::distance(input.name.begin(), found)];
      if (!std::isfinite(value)) { status("INVALID_COMMAND_VALUE"); return; }
      target.push_back(value);
    }
    auto from = state_monitor->getCurrentState();
    from->update();
    from->updateCollisionBodyTransforms();
    moveit::core::RobotState to(*from), sample(*from);
    to.setJointGroupPositions(joint_group_, target);
    to.update();
    if (!to.satisfiesBounds(joint_group_)) { status("BLOCKED_JOINT_BOUNDS"); return; }
    std::vector<double> actual;
    from->copyJointGroupPositions(joint_group_, actual);
    double max_delta = 0.;
    for (std::size_t i = 0; i < target.size(); ++i)
      max_delta = std::max(max_delta, monitor_->getRobotModel()->getJointOfVariable(names[i])->distance(
          &actual[i], &target[i]));
    if (max_delta > max_lead_ + 1e-6) { status("BLOCKED_REFERENCE_LEAD"); return; }
    const auto steps = std::max(1, static_cast<int>(std::ceil(max_delta / path_step_)));
    planning_scene_monitor::LockedPlanningSceneRO locked(monitor_);
    // The read lock covers the shared planning scene for all collision queries.
    const auto scene = monitor_->getPlanningScene();
    ClearancePolicy::Distances distances;
    // A true intersection/contact at the initial state remains blocked. Only
    // positive near-margin clearances can use the geometric retreat policy.
    if (!measureClearance(scene, *from, response, distances)) return;
    ClearancePolicy clearance(distances, {scene_margin_, self_margin_});
    for (int i = 1; i <= steps; ++i)
    {
      from->interpolate(to, static_cast<double>(i) / steps, sample);
      sample.updateCollisionBodyTransforms();
      if (!measureClearance(scene, sample, response, distances)) return;
      if (!clearance.accepts(distances))
      {
        status("BLOCKED_CLEARANCE: scene=" + std::to_string(distances[0]) +
               " self=" + std::to_string(distances[1]));
        return;
      }
    }
    if (!clearance.finishes(max_delta > 1e-8))
    { status("BLOCKED_CLEARANCE_NO_RETREAT"); return; }
    response.valid = true;
    status(clearance.recovering() ? (max_delta > 1e-8 ? "RETREATING_FROM_PROXIMITY" : "HOLDING_IN_PROXIMITY") : "READY");
  }

  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  const moveit::core::JointModelGroup* joint_group_{};
  std::string group_, frame_, floor_id_, last_status_;
  double feedback_timeout_{}, max_lead_{}, path_step_{}, scene_margin_{}, self_margin_{};
  moveit_msgs::msg::PlanningScene floor_scene_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<moveit_msgs::srv::GetStateValidity>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("teleop_collision_guard");
  try { auto guard = std::make_shared<CollisionGuard>(node); rclcpp::spin(node); }
  catch (const std::exception& error) { RCLCPP_FATAL(node->get_logger(), "%s", error.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
  return 0;
}
