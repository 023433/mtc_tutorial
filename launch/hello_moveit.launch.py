from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  # Set MoveIt configuration
  moveit_config = MoveItConfigsBuilder("omx_moveit", package_name="omx_moveit").to_moveit_configs()
  moveit_config.to_dict().update({'use_sim_time' : True})

  x = LaunchConfiguration('x', default='0.15')
  y = LaunchConfiguration('y', default='-0.1')
  z = LaunchConfiguration('z', default='0.3')
  ox = LaunchConfiguration('ox', default='0.0')
  oy = LaunchConfiguration('oy', default='0.0')
  oz = LaunchConfiguration('oz', default='0.0')
  ow = LaunchConfiguration('ow', default='1.0')

  joint1 = LaunchConfiguration('joint1', default='0.0')
  joint2 = LaunchConfiguration('joint2', default='0.0')
  joint3 = LaunchConfiguration('joint3', default='0.0')
  joint4 = LaunchConfiguration('joint4', default='0.0')
  action = LaunchConfiguration('action', default='init')


  # Define hello_moveit node
  hello_moveit_node = Node(
    package="mtc_tutorial",
    executable="hello_moveit",
    parameters=[
      # moveit_config.robot_description,  # Load URDF
      # moveit_config.robot_description_semantic,  # Load SRDF
      moveit_config.robot_description_kinematics,  # Load kinematics.yaml
      {
        "x": x,
        "y": y,
        "z": z,
        "ox": ox,
        "oy": oy,
        "oz": oz,
        "ow": ow,
        "joint1": joint1,
        "joint2": joint2,
        "joint3": joint3,
        "joint4": joint4,
        "action": action,
      }
    ],
    
  )

  return LaunchDescription([hello_moveit_node])