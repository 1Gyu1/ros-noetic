cmake_minimum_required(VERSION 2.8.3)
project(im_pickit_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

add_message_files(
  FILES
    BlacklistedPick.msg
    BlacklistedPickArray.msg
    BoundingBox.msg
    CalibrationStatus.msg
    CalibrationValidationResult.msg
    ColorRangeRGB.msg
    File.msg
    FolderContent.msg
    Model.msg
    ModelClouds.msg
    ModelsClouds.msg
    ModelsData.msg
    MonitoringEvent.msg
    Object.msg
    ObjectArray.msg
    ObjectPickFrame.msg
    ObjectPickFrames.msg
    ObjectPickFrameSymmetry.msg
    PointCloud2Array.msg
    PointIndices.msg
    Reliability.msg
    ReliabilityValue.msg
    SnapshotStatus.msg
    StatusError.msg
    StatusInvalid.msg
    StatusUnpickable.msg
    SubfolderContent.msg
    SubModel.msg
    TeachXDModelsData.msg
    ToolModels.msg
)

add_service_files(
  FILES
    AddBlacklistedPick.srv
    AddMeshTool.srv
    AddShapeModel.srv
    AddSubModel.srv
    BackupConfigurations.srv
    CalculateCalibration.srv
    CaptureImage.srv
    CheckForObjects.srv
    ConfigureCalibration.srv
    ConstructTeachXdModel.srv
    CopyConfiguration.srv
    CreateConfiguration.srv
    DetectTeachXdBoxMasks.srv
    EditConfigurationFiles.srv
    IntArg.srv
    LoadConfig.srv
    LoadSnapshot.srv
    SetMarkersMode.srv
    ModelData.srv
    ModelXdData.srv
    ProcessImage.srv
    RemoveSubModel.srv
    RestoreConfigurations.srv
    SaveActiveConfiguration.srv
    SaveCameraSettings.srv
    SaveConfiguration.srv
    SaveSnapshot.srv
    SetModelData.srv
    SetShapePickFrames.srv
    SetToolModels.srv
    SetTransform.srv
    StrArg.srv
    TeachFromCad.srv
    TeachFromCamera.srv
    TeachFromDXF.srv
    TeachRoi.srv
    ValidateCalibration.srv
)

add_action_files(
  FILES
  GetReferenceFrame.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    shape_msgs
    std_msgs
    std_srvs
    visualization_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    message_runtime
    sensor_msgs
    shape_msgs
    std_msgs
    std_srvs
    visualization_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS
  scripts/pick_and_place_example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
