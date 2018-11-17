cmake_minimum_required(VERSION 2.8.3)
project(cac_msgs)

find_package(catkin REQUIRED COMPONENTS rospy
  std_msgs
  message_generation
  )

 add_message_files(
   FILES
   BB.msg
   BBList.msg
   Person.msg
   PickupPoint.msg
   Pose.msg
   PoseList.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
