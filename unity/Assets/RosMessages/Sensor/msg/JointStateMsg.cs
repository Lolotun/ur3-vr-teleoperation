// Generated for workshop Day 2 (sensor_msgs/JointState).
using System;
using System.Linq;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Sensor
{
    [Serializable]
    public class JointStateMsg : Message
    {
        public const string k_RosMessageName = "sensor_msgs/JointState";
        public override string RosMessageName => k_RosMessageName;

        public Std.HeaderMsg header;
        public string[] name;
        public double[] position;
        public double[] velocity;
        public double[] effort;

        public JointStateMsg()
        {
            header = new Std.HeaderMsg();
            name = Array.Empty<string>();
            position = Array.Empty<double>();
            velocity = Array.Empty<double>();
            effort = Array.Empty<double>();
        }

        public static JointStateMsg Deserialize(MessageDeserializer deserializer) => new JointStateMsg(deserializer);

        private JointStateMsg(MessageDeserializer deserializer)
        {
            header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out name, deserializer.ReadLength());
            deserializer.Read(out position, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out velocity, sizeof(double), deserializer.ReadLength());
            deserializer.Read(out effort, sizeof(double), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(header);
            serializer.WriteLength(name);
            serializer.Write(name);
            serializer.WriteLength(position);
            serializer.Write(position);
            serializer.WriteLength(velocity);
            serializer.Write(velocity);
            serializer.WriteLength(effort);
            serializer.Write(effort);
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
