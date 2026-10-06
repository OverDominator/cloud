# Third party notices

## Unity ROS TCP Connector

The embedded package under `unity/Packages/com.unity.robotics.ros-tcp-connector` was copied from the active local package. On 6 October 2026, all 533 embedded files were compared with the accompanying local `ROS-TCP-Connector-main` package: 529 matched byte-for-byte and four source files differed (OutgoingMessageSender.cs, ROSConnection.cs, RosTopicState.cs and TopicMessageSender.cs under Runtime/TcpConnector). These four files now carry local-modification notices; no executable logic was changed by adding these notices. This establishes comparison with the accompanying local baseline, not an independently verified upstream commit.

The preserved Apache-2.0 LICENSE matches that baseline's licence byte-for-byte. Its SHA-256 is `2e21a5b872a2cdfec6f89db4c93be2867e05b481e77b9c4a6af1caf813129fb0`. Licence, third-party notices and acknowledgements are retained in `third_party/ros_tcp_connector/`.

## External runtime dependencies

FAR Planner, ROS-TCP-Endpoint, the autonomous exploration development environment, Unity registry packages and Depth Anything V2 have their own licensing terms. External source trees, dependency reconstruction patches and model weights are not included. Unresolved external redistribution conditions remain documented in `docs/dependency_sources.json`; publication of this repository does not clear or grant rights to those excluded materials.

## Project material

The ROS package's MIT metadata is not a blanket licence for the entire repository. Scene assets, materials, code ownership and any third-party fragments require a provenance review. No claim of sole authorship or unrestricted redistribution is made by this staging operation.
