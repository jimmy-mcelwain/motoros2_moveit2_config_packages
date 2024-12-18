# MotoROS2 MoveIt2 Setup Instructions
Example MoveIt2 config packages were created for a Yaskawa Motoman GP-12 on a YRC1000 and a Yaskawa Motoman HC10DTP on a YRC1000 Micro. Tested using MotoROS2 Humble. This process was created on Ubuntu 22.04.5

This repository allows for movement using MoveIt2 through Rviz. Other movement methods may require further modifications. 

## Step 1
Before anything else, you must set up MotoROS2. 

### Step 1.1
Go through the [MotoROS2 setup instructions](https://github.com/Yaskawa-Global/motoros2)

### Step 1.2
Go through the [MotoROS2 Client Interface Dependencies](https://github.com/yaskawa-global/motoros2_client_interface_dependencies) setup instructions.

Get to the point where you are able to start the servos with 

`ros2 service call /start_traj_mode motoros2_interfaces/srv/StartTrajMode`

Once you are able to start the servos, proceed to the next step. 

## Step 2
Go through the [instructions for setting up MoveIt](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html). Make sure to reference the correct ROS 2 distribution (e.g. foxy, galactic, humble, etc.) that you are using with MotoROS2. 

## Step 3
Check if you are using a robot model that already has a `motoman_XXXX_moveit2_config` directory within the src directory of this repo. 

If you find a match for your model, then skip to step 7. Otherwise, go [here](https://github.com/gavanderhoorn/motoman/tree/ros2_rebased_and_cleaned) to search for your robot model's support directory. 

If you find the support package (`motoman_XXXX_support`) in the linked repo, place it in the src directory in this repo. If the corresponding directory does not exist in the linked repo, you will be unable to use MoveIt2 for the time being. Please post your robot model and request in the discussion and we will make an effort to upload the `motoman_XXXX_support` directory for that model. 

**Special note for HC10, GP180, and ES robot models: There are multiple urdf files/meshes in the support directories corresponding to these models. Each different 'version' for these models needs its own moveit2 config, make sure that you choose the files that align with your specific model as you go through this process. Additionally, make sure that if you use a pre-existing moveit2 config (e.g. motoman_hc10dtp_b00_moveit2_config), that this aligns with your specific version of your robot model**

## Step 4
Enter your robot's support directory that you just copied to this repo, and find its `urdf` directory. Note that for this demo, all joint names and link names were changed from `link_1_s`, `joint_1_s`, etc. to `group_1/link_1`, `group_2/joint_1`, etc. Joint names can be configured in `motoros2_config.yaml`, so you may either configure them there or in the `xacro`/`urdf`, but it is essential that the joint names match between what is being published to `/joint_states` by the robot and what the `urdf`/`srdf` are calling the joints. 

If you choose to change the names in the `xacro`/`urdf`, do so and save it. Note that you can edit the contents of the `XXXX.xacro` file to add a prefix to all link and joint names. 

If you choose to change the joint names on the controller side to match the `urdf`, learn how to do so [here](https://github.com/Yaskawa-Global/motoros2/blob/main/doc/faq.md#can-names-of-joints-be-changed). 

Regardless of which method you choose, after you make the changes, open up a terminal in the repo's root directory and run the following commands to generate a `urdf`, replacing the 'XXXX' with your own model:

1. Install all prerequisites/dependencies

`rosdep install --from-paths src --ignore-src`

2. Build the package

`colcon build`

3. Source the package

`source install/local_setup.bash`

4. Change directory to the `urdf` directory

`cd src/[robot support directory]/urdf`

5. Generate the `urdf` file

`xacro XXXX.xacro > XXXX.urdf`

## Step 5

After MoveIt is set up, you will use the [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html) in order to generate some required files. 

Note that you will need to edit many of the files generated by the setup assistant, but it is the easiest way to generate some of the files which MoveIt2 needs, e.g. `srdf` files. If you choose, you may manually edit the files available in this repo instead to get it to match your robot's specifications, but we do not recommend that method, especially to newcomers. 

### Step 5.1
Follow step 1 of the MoveIt Setup Assistant (Step 1: Start), replacing the `urdf` file in the examples with your own `urdf` file that you just generated. 

### Step 5.2
Follow step 2 of the MoveIt Setup Assistant (Step 2: Generate Self-Collision Matrix)

![Collision matrix](/assets/screenshots/collision_matrix.png)

### Step 5.3
***Skip*** step 3 of the MoveIt Setup Assistant (Step 3: Add Virtual Joints). 

### Step 5.4
We will make some modifications to step 4 of the MoveIt Setup Assistant (Step 4: Add Planning Groups). Click on "Planning Groups" in the left pane and click "Add Group". Name the group whatever you want (e.g. "manipulator"), choose `kdl_kinematics_plugin/KDLKinematicsPlugin`, and ***click on "Add Kin. Chain" instead of "Add Joints"***. Expand the 'Robot Links' tree as far as possible. Highlight your base link (e.g. `group_1/base_link`) and click 'Choose Selected' next to the 'Base Link' box. Then highlight your tip link on the list (e.g. `group_1/tool0`) and click 'Choose Selected' next to the 'Tip Link' box. See the screenshot below for an example. 

![Planning Group Process](/assets/screenshots/planning_group_process.png)

Then save the results. Your screen should be similar to the screenshot below. 

![Planning Group Results](/assets/screenshots/planning_group_result.png)

### Step 5.5
You may perform Step 5: Add Robot Poses, ignoring the end effector portion. Set up any poses that you want, e.g. a home pose with all 0s set. 

### Step 5.6
***Skip*** step 6 of the MoveIt Setup Assistant (Step 6: Label End Effectors). 

### Step 5.7
***Skip*** step 7 of the MoveIt Setup Assistant (Step 7: Add Passive Joints). 

### Step 5.8
***Skip*** step 8 of the MoveIt Setup Assistant (Step 8: ros2_control URDF Modification). 

### Step 5.9
***Skip*** step 9 of the MoveIt Setup Assistant (Step 9: ROS 2 Controllers). 

### Step 5.10
We will perform step 10 (Setup MoveIt Controllers) of the MoveIt Setup Assistant with some modifications. First, click on MoveIt Controllers in the left pane. Click "Add Controller". Set "Controller Name" to `follow_joint_trajectory`, set "Controller Type" to FollowJointTrajectory, make sure that "Action Namespace" is blank (unless you configured MotoROS2 to namespace its action server, in which case you will set it to the namespace), and set "Default" to true. 

![Follow Joint Trajectory Process](/assets/screenshots/follow_joint_trajectory_process.png)

Then add the planning joint groups. Click on your joint group (e.g 'manipulator') and hit the `>` to add it. Then click save. 

![Follow Joint Trajectory Group](/assets/screenshots/follow_joint_trajectory_group.png)

The resultant screen should look like this:

![Follow Joint Trajectory Result](/assets/screenshots/follow_joint_trajectory_result.png)

### Step 5.11
***Skip*** step 11 of the MoveIt Setup Assistant (Step 11: Perception). 

### Step 5.12
For step 12 of the MoveIt Setup Assistant (Step 12: Launch Files), we recommend only generating the "Demo Launch" and "RViz Launch and Config" files, but if you would like to generate others you may do so. They will likely need further modification to work properly. 

![Launch Files](/assets/screenshots/launch_files.png)

### Step 5.13
Follow step 13 of the MoveIt Setup Assistant (Step 13: Add Author Information). 

### Step 5.14
In step 14: Generate Configuration Files, go the the Configuration Files tab. Click on the 'Browse' button in 'Configuration Package Save Path. Create a new folder inside of the src directory that matches the naming convention of `motoman_XXXX_moveit2_config`. Select that directory as where the files will be generated. 

Afterwards, select the files as shown in the screenshot below. Ignore any pop-ups while deselecting unnecessary files. 

![Generated file selection](/assets/screenshots/configuration_files.png)

Finally, click the 'Generate Package' button in the bottom right. Click 'OK' if you get a pop-up saying that some steps were skipped, that is okay. 

After the package is generated, exit the setup assistant. 

## Step 6
After generating the package in MoveIt Setup Assistant you will have to make many modifications to files that were improperly generated. 

### Step 6.1
Go to the newly generated `motoman_XXXX_moveit2_config` package and edit the `demo.launch.py`. Copy and paste the contents of the `demo.launch.py` from the `motoman_gp12_moveit2_config` package. Replace all references to the 'gp12' specific files with references to your robot files. 

### Step 6.2
In your `motoman_XXXX_moveit2_config` package and edit the `package.xml`. Copy and paste the `<exec_depends>` section of the `package.xml` from the `motoman_gp12_moveit2_config` package. Replace the reference to `motoman_gp12_support` with the support package associated with your robot model. 

### Step 6.3
Go to `moveit_controllers.yaml` and replace the contents of the `action_ns` with and empty string `""` (unless you configured MotoROS2 to namespace its action server, in which case you will set it to the namespace). 

### Step 6.4
Go to `motoman_XXXX.urdf.xacro` and delete the elements below the "Import control_xacro" line. The resultant file should look something like this:

```
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="motoman_hc10dtp_b00">
    <xacro:arg name="initial_positions_file" default="initial_positions.yaml" />

    <!-- Import motoman_hc10dtp_b00 urdf file -->
    <xacro:include filename="$(find motoman_hc10_support)/urdf/hc10dtp_b00.urdf" />

</robot>
```

### Step 6.5
Go to `.setup_assistant` and delete the elements at and below the "Import control_xacro" line. The resulting file should look something like this:

```
moveit_setup_assistant_config:
  urdf:
    package: motoman_hc10_support
    relative_path: urdf/hc10dtp_b00.urdf
  srdf:
    relative_path: config/motoman_hc10dtp_b00.srdf
  package_settings:
    author_name: "XXXX"
    author_email: "YYYY@ZZZ.com"
    generated_timestamp: 1730388944
```

## Step 7
Navigate to the root directory of the repo in your terminal and execute the following commands. 
1. Install all prerequisites/dependencies
`rosdep install --from-paths src --ignore-src`
2. Build the package
`colcon build`
3. Source the package
`source install/local_setup.bash`
4. Launch the Rviz MoveIt2 Demo. You may optionally include the warehouse_host argument if you want to save scenes and states
`ros2 launch motoman_XXXX_moveit2_config demo.launch.py [warehouse_host:=[file name].sqlite]`

## Checking if it works

To check if it works, manually jog the robot to a position other than home. 

Afterwards, switch to remote mode. 

Then, open a new terminal, source the appropriate setup script (make sure that you have motoros2_interfaces, industrial_messages, and the ROS2 installation sourced), and call 

`ros2 service call /start_traj_mode motoros2_interfaces/srv/StartTrajMode`

Then view the RViz screen from the earlier launch. Go to the planning tab and change the 'Start State' to '\<current\>' and goal state to 'home'. 

The robot will move at the next action, so prepare to press the E-Stop in case something goes wrong. 

Then click the 'Plan & Execute' button and the robot should move. If it does not move, ensure that your servos are on. If they are, look in the terminal that launched RViz, and read through the error messages. 

Note that there are RViz issues where "incorrect" errors are reported to the RViz terminal, so some of the errors can be ignored. 
Examples include:

`[move_group-1] [ERROR] [1730395155.718377952] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates`

`[move_group-1] [ERROR] [1730395155.737128288] [moveit.ros_planning.planning_pipeline]: Exception while loading planner 'chomp_interface/CHOMPPlanner': According to the loaded plugin descriptions the class chomp_interface/CHOMPPlanner with base class type planning_interface::PlannerManager does not exist. Declared types are  ompl_interface/OMPLPlanner pilz_industrial_motion_planner/CommandPlannerAvailable plugins: ompl_interface/OMPLPlanner, pilz_industrial_motion_planner/CommandPlanner`

`[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.`

`[rviz2-2] [ERROR] [1730395159.562577139] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available`

These errors can be safely ignored, but other ones may result from real problems with your setup. 

If you have completed the process and everything is working smoothly, then it would be appreciated if you could create a PR that would add your new `motoman_xxxx_moveit2_config` package to the repo for others to use. 

