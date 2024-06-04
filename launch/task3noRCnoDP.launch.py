import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="dis_tutorial7",
            executable="arm_mover_actions.py",
            name="arm_mover_actions"
        ),
        launch_ros.actions.Node(
            package="RINS-task-3",
            executable="image_gatherer.py",
            name="image_gatherer"
        ),
        launch_ros.actions.Node(
            package="RINS-task-3",
            executable="cylinder_segmentation",
            name="cylinder_segmentation"
        ),
        launch_ros.actions.Node(
            package="RINS-task-3",
            executable="detect_rings.py",
            name="detect_rings"
        )
    ])