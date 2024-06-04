import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
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
            executable="detect_people.py",
            name="detect_people"
        ),
        launch_ros.actions.Node(
            package="RINS-task-3",
            executable="detect_rings.py",
            name="detect_rings"
        )
    ])