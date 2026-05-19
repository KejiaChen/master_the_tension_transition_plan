import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def sanitized_ld_library_path():
    conda_markers = ("anaconda", "miniconda", "mambaforge", "micromamba")
    entries = os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
    filtered_entries = [
        entry for entry in entries
        if entry and not any(marker in entry.lower() for marker in conda_markers)
    ]
    return os.pathsep.join(filtered_entries)


def generate_launch_description():
    # Declare the scene file argument
    scene_file_arg = DeclareLaunchArgument(
        "scene_file",
        default_value=os.path.expanduser("~/ws_humble/scene/mongodb_8.scene"),
        description="Path to the .scene file to be loaded",
    )

    mesh_file_arg = DeclareLaunchArgument(
        "mesh_file",
        default_value=os.path.expanduser("~/ws_humble/scene/surface.STL"),
        description="Path to the mesh file to be loaded",
    )

    clip_file_arg = DeclareLaunchArgument(
        "clip_file",
        default_value="",  # Default file path
        description="Optional path to the clip configuration file to be loaded",
    )

    use_qb_board_coordinate = DeclareLaunchArgument(
        "use_qb_board_coordinate",
        default_value="true",
        description="Whether to use QB board coordinate for pose reading of clip_file.",
    )

    use_sensone_left = DeclareLaunchArgument(
        "use_sensone_left",
        default_value="false",
        description="Whether to include the BotaSys external force torque sensor in the left robot model",
    )

    alter_finger_left = DeclareLaunchArgument(
        "alter_finger_left",
        default_value="true",
        description="Whether to alter the left robot model's finger configuration",
    )

    use_sensone_right = DeclareLaunchArgument(
        "use_sensone_right",
        default_value="true",
        description="Whether to include the BotaSys external force torque sensor in the right robot model",
    )

    alter_finger_right = DeclareLaunchArgument(
        "alter_finger_right",
        default_value="false",
        description="Whether to alter the right robot model's finger configuration",
    )

    add_clip_hats = DeclareLaunchArgument(
        "add_clip_hats",
        default_value="false",
        description="Whether to include extra hats in each clip fixture",
    )

    
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda")
        .robot_description(file_path="config/panda.urdf.xacro",
                            mappings={
                                    # "ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type"),
                                    "use_sensone_left": LaunchConfiguration("use_sensone_left"),
                                    "alter_finger_left": LaunchConfiguration("alter_finger_left"),
                                    "use_sensone_right": LaunchConfiguration("use_sensone_right"),
                                    "alter_finger_right": LaunchConfiguration("alter_finger_right"),
                        },
        )
        .robot_description_semantic(
            file_path="config/panda.srdf.xacro",
            mappings={
                "use_sensone_left": LaunchConfiguration("use_sensone_left"),
                "alter_finger_left": LaunchConfiguration("alter_finger_left"),
                "use_sensone_right": LaunchConfiguration("use_sensone_right"),
                "alter_finger_right": LaunchConfiguration("alter_finger_right"),
            },
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


    # MTC Demo node
    load_scene= Node(
        package="master_the_tension_transition_plan",
        executable="load_scene",
        # package="moveit2_tutorials",
        # executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
        arguments=[LaunchConfiguration("scene_file"),
                   LaunchConfiguration("mesh_file"),
                   LaunchConfiguration("clip_file"),  # Pass clip_file even if it's empty
                   LaunchConfiguration("use_qb_board_coordinate"),
                   LaunchConfiguration("add_clip_hats")
        ],
    )
    
    dlo_model = Node(
    package="master_the_tension_transition_plan",
    executable="dlo_line_model",
    output="screen",
    parameters=[
        moveit_config.to_dict(),
    ],
    )
    
    return LaunchDescription([SetEnvironmentVariable("LD_LIBRARY_PATH", sanitized_ld_library_path()),
                              use_sensone_left, 
                              alter_finger_left,
                              use_sensone_right, 
                              alter_finger_right,
                              scene_file_arg, 
                              mesh_file_arg,
                              clip_file_arg,
                              use_qb_board_coordinate,
                              add_clip_hats,
                              load_scene])
