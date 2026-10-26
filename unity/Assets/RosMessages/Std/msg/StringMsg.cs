// Generated for workshop Day 3 (std_msgs/String).
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    [Serializable]
    public class StringMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/String";
        public override string RosMessageName => k_RosMessageName;

        public string data;

        public StringMsg()
        {
            data = "";
        }

        public StringMsg(string data)
        {
            this.data = data;
        }

        public static StringMsg Deserialize(MessageDeserializer deserializer) => new StringMsg(deserializer);

        private StringMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out data);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(data);
        }

        public override string ToString()
        {
            return "StringMsg: data=" + data;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
