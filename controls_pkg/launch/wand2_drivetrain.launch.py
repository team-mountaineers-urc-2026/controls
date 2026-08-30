from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    FRONT_LEFT_ID = 0x141
    FRONT_RIGHT_ID = 0x143

    BACK_LEFT_ID = 0x142
    BACK_RIGHT_ID = 0x144

    
    ld = LaunchDescription()

    ld.add_action(
        launch_ros.actions.Node(
            package='controls_pkg',
            executable='drivetrain_debugger',
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            package='controls_pkg',
            executable='drivetrain',
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/front_left", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': FRONT_LEFT_ID}]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/front_right", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': FRONT_RIGHT_ID}]  
        )
    )
        
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/back_left", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': BACK_LEFT_ID}]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/back_right", 
            package='controls_pkg',
            executable='motor',
            parameters=[{'arbitration_id': BACK_RIGHT_ID}]  
        )
    )
    
    return ld
