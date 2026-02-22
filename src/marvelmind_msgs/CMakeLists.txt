cmake_minimum_required(VERSION 3.8)
project(marvelmind_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BeaconDistance.msg"
  "msg/BeaconPosA.msg"
  "msg/HedgeImuFusion.msg"
  "msg/HedgeImuRaw.msg"
  "msg/HedgePos.msg"
  "msg/HedgePosA.msg"
  "msg/HedgePosAng.msg"
  "msg/HedgeQuality.msg"
  "msg/HedgeTelemetry.msg"
  "msg/MarvelmindUserData.msg"
  "msg/MarvelmindWaypoint.msg"
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
