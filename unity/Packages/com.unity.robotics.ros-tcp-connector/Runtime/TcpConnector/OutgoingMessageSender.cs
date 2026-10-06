// Locally modified for the Unity-ROS fire-rescue integration.
// Differs from the accompanying ROS-TCP-Connector-main baseline; marked 2026-10-06.
// See third_party/ros_tcp_connector/LICENSE and THIRD_PARTY_NOTICES.md at repository root.
using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.ROSTCPConnector
{
    public abstract class OutgoingMessageSender
    {

        public enum SendToState
        {
            Normal,
            NoMessageToSendError,
            QueueFullWarning
        }

        public abstract SendToState SendInternal(MessageSerializer m_MessageSerializer, System.IO.Stream stream);

        public abstract void ClearAllQueuedData();
    }

    /**
     * Simple implementation of a OutgoingMessageSender that is used for sys commands
     * as they are handled differently to typical ROS messages and sent as JSON strings.
     */
    public class SysCommandSender : OutgoingMessageSender
    {
        List<byte[]> m_ListOfSerializations;

        public SysCommandSender(List<byte[]> listOfSerializations)
        {
            m_ListOfSerializations = listOfSerializations;
        }

        public override SendToState SendInternal(MessageSerializer m_MessageSerializer, Stream stream)
        {
            foreach (byte[] statement in m_ListOfSerializations)
            {
                stream.Write(statement, 0, statement.Length);
            }

            return SendToState.Normal;
        }

        public override void ClearAllQueuedData()
        {
            m_ListOfSerializations.Clear();
        }
    }

    // A ROS service request is a two-frame protocol transaction: the
    // __service_request command must be followed immediately by the request
    // payload.  Queuing those frames independently lets a high-rate topic
    // (for example /odom) slip between them, after which the endpoint treats
    // that topic as the service destination.  Keep both frames in one sender
    // so the connection thread writes them atomically.
    public class ServiceRequestSender : OutgoingMessageSender
    {
        readonly List<byte[]> m_CommandSerializations;
        readonly string m_ServiceTopic;
        Message m_Request;

        public ServiceRequestSender(
            List<byte[]> commandSerializations,
            string serviceTopic,
            Message request)
        {
            m_CommandSerializations = commandSerializations;
            m_ServiceTopic = serviceTopic;
            m_Request = request;
        }

        public override SendToState SendInternal(
            MessageSerializer messageSerializer,
            Stream stream)
        {
            foreach (byte[] statement in m_CommandSerializations)
                stream.Write(statement, 0, statement.Length);

            messageSerializer.Clear();
            messageSerializer.Write(m_ServiceTopic);
            messageSerializer.SerializeMessageWithLength(m_Request);
            messageSerializer.SendTo(stream);
            m_Request = null;
            return SendToState.Normal;
        }

        public override void ClearAllQueuedData()
        {
            m_CommandSerializations.Clear();
            m_Request = null;
        }
    }
}
