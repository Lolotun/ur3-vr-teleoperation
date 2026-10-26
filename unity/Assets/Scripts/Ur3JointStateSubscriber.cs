using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
public class Ur3JointStateSubscriber : MonoBehaviour {
 public string topic="/joint_states"; public Transform robotRoot;
 static readonly Dictionary<string,string> M=new(){{"shoulder_link","shoulder_pan_joint"},{"upper_arm_link","shoulder_lift_joint"},{"forearm_link","elbow_joint"},{"wrist_1_link","wrist_1_joint"},{"wrist_2_link","wrist_2_joint"},{"wrist_3_link","wrist_3_joint"}};
 readonly Dictionary<string,Transform> links=new(); readonly Dictionary<string,Quaternion> rests=new(); readonly Dictionary<string,float> values=new(); bool pending;
 void Awake(){if(robotRoot==null)robotRoot=transform;foreach(Transform t in robotRoot.GetComponentsInChildren<Transform>(true))if(M.TryGetValue(t.name,out var n)){links[n]=t;rests[n]=t.localRotation;}}
 void Start()=>ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(topic,OnState);
 void OnState(JointStateMsg m){for(int i=0;i<Mathf.Min(m.name.Length,m.position.Length);i++)if(links.ContainsKey(m.name[i]))values[m.name[i]]=(float)m.position[i];pending=true;}
 void Update(){if(!pending)return;foreach(var v in values){var t=links[v.Key];var b=t.GetComponent<ArticulationBody>();var a=b!=null?b.anchorRotation:Quaternion.identity;t.localRotation=rests[v.Key]*(a*Quaternion.Euler(v.Value*Mathf.Rad2Deg,0,0)*Quaternion.Inverse(a));}pending=false;}
}
