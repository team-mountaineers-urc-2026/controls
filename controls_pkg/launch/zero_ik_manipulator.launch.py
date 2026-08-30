from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    
    ld = LaunchDescription()

    ld.add_action(
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
        )
    )   

    ld.add_action(
        launch_ros.actions.Node(
            package='controls_pkg',
            executable='manipulator_debugger',
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package='controls_pkg',
            executable='zero_ik_manipulator',
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            package='controls_pkg',
            executable='can',
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/shoulder", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': 0x146}]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/elbow", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': 0x145}]  
        )
    )
        
    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/wrist_roll", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': 0x149}]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/wrist_pitch", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': 0x148}]  
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            namespace="manipulator/linear_rail", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': 0x147}]  
        )
    )
    
    return ld
