# Change Log

## v1.5.0 (2018-11-26)

### Added
- Add torobo_gazebo_ros_control package in order to avoid SetPosition jump problem of prismatic joint
- Add torobo_gripper_action_controller package in order to avoid crash when preempting gripper action

### Changed
- Change ros_control plugin from gazebo_ros_control to torobo_gazebo_ros_control written on torobo_description/urdf/torobo.gazebo
- Change gripper_controller's type from GripperActionController to ToroboGripperActionController written on torobo_description/config/.../controllers.yaml

### Fixed
- Fix a bug in ToroboJointController: '<New>' item in TP list is removed when 'Enter' key is pressed when '<New>' item is selected in TP list. 

## v1.4.0 (2018-10-29)
### Changed
- Perform refactoring of the function to load rosparam dynamically at initialization
- Separate URDF of base from robot's URDF
