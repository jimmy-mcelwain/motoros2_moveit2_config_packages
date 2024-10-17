import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
            
      moveit_config = (
            MoveItConfigsBuilder("motoman_gp12", 
			      package_name="motoman_gp12_moveit2_config",)
        	  .robot_description(file_path="config/motoman_gp12.urdf.xacro")
        	  .robot_description_semantic(file_path="config/motoman_gp12.srdf")
    		    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    		    .to_moveit_configs()
	  )
    

	  # Start the actual move_group node/action server
      move_group_node = Node(
        
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
            arguments=["--ros-args", "--log-level", "info"],
	  )

	  
      # RViz
      rviz_config_path = os.path.join(
        get_package_share_directory("motoman_gp12_moveit2_config"), "config", "moveit.rviz"
      )


      rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
      )      

      
      return LaunchDescription(
        [
          move_group_node, 
          rviz_node, 
        ]
      )