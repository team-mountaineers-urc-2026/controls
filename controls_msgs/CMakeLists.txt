cmake_minimum_required(VERSION 3.8)
project(controls_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WritePIDParamsROMMsgRecvParams.msg"
  "msg/SpeedClosedLoopControlMsgRecvParams.msg"
  "msg/CommunicationBaudRateSettingMsgRecvParams.msg"
  "msg/CommunicationInterruptionProtectionTimeSettingMsgRecvParams.msg"
  "msg/WriteAccelerationRAMROMMsgSentParams.msg"
  "msg/CommunicationBaudRateSettingMsgSentParams.msg"
  "msg/SingleTurnPositionControlMsgRecvParams.msg"
  "msg/ReadAccelerationMsgSentParams.msg"
  "msg/AbsolutePositionClosedLoopControlMsgRecvParams.msg"
  "msg/SystemRuntimeReadMsgRecvParams.msg"
  "msg/MotorPowerAcquisitionMsgSentParams.msg"
  "msg/SystemSoftwareVersionDateReadMsgRecvParams.msg"
  "msg/TorqueClosedLoopControlMsgRecvParams.msg"
  "msg/CommunicationInterruptionProtectionTimeSettingMsgSentParams.msg"
  "msg/SystemBrakeReleaseMsgSentParams.msg"
  "msg/SystemSoftwareVersionDateReadMsgSentParams.msg"
  "msg/SystemBrakeReleaseMsgRecvParams.msg"
  "msg/ReadMotorStatus2MsgSentParams.msg"
  "msg/ReadMultiTurnEncoderZeroOffsetMsgSentParams.msg"
  "msg/SystemRuntimeReadMsgSentParams.msg"
  "msg/ReadPIDParamsMsgSentParams.msg"
  "msg/CANIDSettingMsgRecvParams.msg"
  "msg/WriteCurrentMultiTurnZeroMsgSentParams.msg"
  "msg/ReadSingleTurnAngleMsgRecvParams.msg"
  "msg/MotorPowerAcquisitionMsgRecvParams.msg"
  "msg/SystemOperatingModeAcquisitionMsgSentParams.msg"
  "msg/CanMessage.msg"
  "msg/SystemResetMsgRecvParams.msg"
  "msg/SystemResetMsgSentParams.msg"
  "msg/SpeedClosedLoopControlMsgSentParams.msg"
  "msg/ReadMotorStatus3MsgSentParams.msg"
  "msg/WriteAccelerationRAMROMMsgRecvParams.msg"
  "msg/IncrementalPositionClosedLoopControlMsgSentParams.msg"
  "msg/ReadMultiTurnEncoderOriginalPositionMsgSentParams.msg"
  "msg/MotorShutdownMsgRecvParams.msg"
  "msg/IncrementalPositionClosedLoopControlMsgRecvParams.msg"
  "msg/ReadMotorStatus2MsgRecvParams.msg"
  "msg/CANIDSettingMsgSentParams.msg"
  "msg/ReadMotorStatus1MsgRecvParams.msg"
  "msg/SystemBrakeLockMsgRecvParams.msg"
  "msg/MotorStopMsgSentParams.msg"
  "msg/TorqueClosedLoopControlMsgSentParams.msg"
  "msg/ReadMultiTurnAngleMsgRecvParams.msg"
  "msg/SystemBrakeLockMsgSentParams.msg"
  "msg/WritePIDParamsROMMsgSentParams.msg"
  "msg/WriteCurrentMultiTurnZeroMsgRecvParams.msg"
  "msg/AbsolutePositionClosedLoopControlMsgSentParams.msg"
  "msg/ReadAccelerationMsgRecvParams.msg"
  "msg/ReadPIDParamsMsgRecvParams.msg"
  "msg/ReadSingleTurnAngleMsgSentParams.msg"
  "msg/SystemOperatingModeAcquisitionMsgRecvParams.msg"
  "msg/ReadMultiTurnEncoderPositionMsgRecvParams.msg"
  "msg/MotorModelReadingMsgSentParams.msg"
  "msg/ReadMultiTurnEncoderOriginalPositionMsgRecvParams.msg"
  "msg/ReadMultiTurnAngleMsgSentParams.msg"
  "msg/ReadMultiTurnEncoderZeroOffsetMsgRecvParams.msg"
  "msg/ReadMotorStatus1MsgSentParams.msg"
  "msg/ReadSingleTurnEncoderMsgRecvParams.msg"
  "msg/WriteEncoderMultiTurnZeroMsgSentParams.msg"
  "msg/WritePIDParamsRAMMsgSentParams.msg"
  "msg/ReadMultiTurnEncoderPositionMsgSentParams.msg"
  "msg/SingleTurnPositionControlMsgSentParams.msg"
  "msg/MotorShutdownMsgSentParams.msg"
  "msg/UartMessage.msg"
  "msg/ReadSingleTurnEncoderMsgSentParams.msg"
  "msg/WriteEncoderMultiTurnZeroMsgRecvParams.msg"
  "msg/ReadMotorStatus3MsgRecvParams.msg"
  "msg/MotorStopMsgRecvParams.msg"
  "msg/WritePIDParamsRAMMsgRecvParams.msg"
  "msg/MotorModelReadingMsgRecvParams.msg"
  "msg/ManipulatorControl.msg"

  
  "srv/CanSendRecv.srv"
  "srv/UartSendRecv.srv"
  "srv/SystemOperatingModeAcquisitionMsg.srv"
  "srv/CommunicationBaudRateSettingMsg.srv"
  "srv/AbsolutePositionClosedLoopControlMsg.srv"
  "srv/SystemBrakeLockMsg.srv"
  "srv/TorqueClosedLoopControlMsg.srv"
  "srv/SystemRuntimeReadMsg.srv"
  "srv/ReadPIDParamsMsg.srv"
  "srv/WritePIDParamsRAMMsg.srv"
  "srv/IncrementalPositionClosedLoopControlMsg.srv"
  "srv/ReadSingleTurnEncoderMsg.srv"
  "srv/MotorStopMsg.srv"
  "srv/ReadMotorStatus3Msg.srv"
  "srv/ReadAccelerationMsg.srv"
  "srv/WritePIDParamsROMMsg.srv"
  "srv/SystemBrakeReleaseMsg.srv"
  "srv/ReadMultiTurnEncoderOriginalPositionMsg.srv"
  "srv/ReadMultiTurnEncoderPositionMsg.srv"
  "srv/ReadMotorStatus2Msg.srv"
  "srv/MotorPowerAcquisitionMsg.srv"
  "srv/MotorShutdownMsg.srv"
  "srv/CommunicationInterruptionProtectionTimeSettingMsg.srv"
  "srv/SystemSoftwareVersionDateReadMsg.srv"
  "srv/ReadMultiTurnAngleMsg.srv"
  "srv/SystemResetMsg.srv"
  "srv/ReadMotorStatus1Msg.srv"
  "srv/WriteCurrentMultiTurnZeroMsg.srv"
  "srv/MotorModelReadingMsg.srv"
  "srv/CANIDSettingMsg.srv"
  "srv/SpeedClosedLoopControlMsg.srv"
  "srv/WriteAccelerationRAMROMMsg.srv"
  "srv/SingleTurnPositionControlMsg.srv"
  "srv/ReadSingleTurnAngleMsg.srv"
  "srv/ReadMultiTurnEncoderZeroOffsetMsg.srv"
  "srv/WriteEncoderMultiTurnZeroMsg.srv"
  DEPENDENCIES std_msgs 
)

ament_package()
