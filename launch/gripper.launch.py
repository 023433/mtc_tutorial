from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  position = LaunchConfiguration('position', default='0.15')

  # Define hello_moveit node
  hello_moveit_node = Node(
    package="mtc_tutorial",
    executable="gripper",
    parameters=[
      {
        "position": position,
      }
    ],
    
  )

  return LaunchDescription([hello_moveit_node])