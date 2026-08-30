from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():

    ld = LaunchDescription()

    front_left_id   = DeclareLaunchArgument(name = 'front_left_id',         default_value = f'{int(0x144)}',   description = 'Motor ID for Front Left Motor in Base 10')
    front_right_id  = DeclareLaunchArgument(name = 'front_right_id',        default_value = f'{int(0x142)}',   description = 'Motor ID for Front Right Motor in Base 10')
    back_left_id    = DeclareLaunchArgument(name = 'back_left_id',          default_value = f'{int(0x143)}',   description = 'Motor ID for Back Left Motor in Base 10')
    back_right_id   = DeclareLaunchArgument(name = 'back_right_id',         default_value = f'{int(0x141)}',   description = 'Motor ID for Back Right Motor in Base 10')
    can_network_id  = DeclareLaunchArgument(name = 'can_network_id',        default_value = 'can0',            description = 'What network are you using?')

    ld.add_action(front_left_id)
    ld.add_action(front_right_id)
    ld.add_action(back_left_id)
    ld.add_action(back_right_id)
    ld.add_action(can_network_id)

    fl_id = LaunchConfiguration('front_left_id')
    fr_id = LaunchConfiguration('front_right_id')
    bl_id = LaunchConfiguration('back_left_id')
    br_id = LaunchConfiguration('back_right_id')
    cn_id = LaunchConfiguration('can_network_id')

    
    ld.add_action(
        launch_ros.actions.Node(
            namespace='drivetrain',
            package='controls_pkg',
            executable='drivetrain',
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/front_left", 
            package='controls_pkg',
            executable='motor',
            parameters=[
                {
                    'arbitration_id': fl_id,
                    'can_network_id': cn_id
                }
            ]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/front_right", 
            package='controls_pkg',
            executable='motor',
            parameters=[
                {
                    'arbitration_id': fr_id,
                    'can_network_id': cn_id
                }
            ]  
        )
    )
        
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/back_left", 
            package='controls_pkg',
            executable='motor',
            parameters=[
                {
                    'arbitration_id': bl_id,
                    'can_network_id': cn_id
                }
            ]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="drivetrain/back_right", 
            package='controls_pkg',
            executable='motor',
            parameters=[
                {
                    'arbitration_id': br_id,
                    'can_network_id': cn_id
                }
            ]  
        )
    )
    
    return ld
