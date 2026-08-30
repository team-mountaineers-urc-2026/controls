from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    
    ld = LaunchDescription()

    shoulder_id     = DeclareLaunchArgument(name = 'shoulder_id',       default_value = f'{int(0x145)}',    description = 'Motor ID for Shoulder Motor in Base 10')
    elbow_id        = DeclareLaunchArgument(name = 'elbow_id',          default_value = f'{int(0x14A)}',    description = 'Motor ID for Elbow Motor in Base 10')
    wrist_pitch_id  = DeclareLaunchArgument(name = 'wrist_pitch_id',    default_value = f'{int(0x14D)}',    description = 'Motor ID for Scoop Drum Motor in Base 10')
    # wrist_roll_id   = DeclareLaunchArgument(name = 'wrist_roll_id',     default_value = f'{int(0x149)}',    description = 'Motor ID for Wrist Roll Motor in Base 10')
    rail_id         = DeclareLaunchArgument(name = 'rail_id',           default_value = f'{int(0x147)}',    description = 'Motor ID for Linear Rail Motor in Base 10')
    # gripper_id         = DeclareLaunchArgument(name = 'gripper_id',           default_value = f'{int(0x14C)}',    description = 'Motor ID for Gripper Motor in Base 10')
    joy_config      = DeclareLaunchArgument(name = 'joy_config',        description = 'Path for the Joy Configuration, must be set on runtime.')
    can_network_id  = DeclareLaunchArgument(name = 'can_network_id',    default_value = 'can3',             description = 'What network are you using?')

    ld.add_action(shoulder_id)
    ld.add_action(elbow_id)
    ld.add_action(wrist_pitch_id)
    # ld.add_action(wrist_roll_id)
    ld.add_action(rail_id)
    # ld.add_action(gripper_id)
    ld.add_action(joy_config)
    ld.add_action(can_network_id)

    sh_id = LaunchConfiguration('shoulder_id')
    el_id = LaunchConfiguration('elbow_id')
    wp_id = LaunchConfiguration('wrist_pitch_id')
    # wr_id = LaunchConfiguration('wrist_roll_id')
    ra_id = LaunchConfiguration('rail_id')
    # gr_id = LaunchConfiguration('gripper_id')
    cn_id = LaunchConfiguration('can_network_id')
    joy_file = LaunchConfiguration('joy_config')

    ld.add_action(
        launch_ros.actions.Node(
            namespace="science_manipulator",
            package='controls_pkg',
            executable='science_manipulator',
            parameters=[
                joy_file,
                {'max_dps': 30}
            ]
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="science_manipulator/shoulder", 
            package='controls_pkg',
            executable='motor',
            parameters=[
                {
                    'arbitration_id': sh_id,
                    'can_network_id' : cn_id
                }
            ]  
        )
    )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="science_manipulator/elbow", 
            package='controls_pkg',
            executable='motor',
            parameters=[
                {
                    'arbitration_id': el_id,
                    'can_network_id' : cn_id
                }
            ]  
        )
    )
        
    # ld.add_action(
    #     launch_ros.actions.Node(
    #         namespace="manipulator/wrist_roll", 
    #         package='controls_pkg',
    #         executable='motor',
    #         parameters=[
    #             {
    #                 'arbitration_id': wr_id,
    #                 'can_network_id' : cn_id
    #             }
    #         ]  
    #     )
    # )
    
    ld.add_action(
        launch_ros.actions.Node(
            namespace="science_manipulator/wrist_pitch", 
            package='controls_pkg',
            executable='motor',
            parameters=[
                {
                    'arbitration_id': wp_id,
                    'can_network_id' : cn_id
                }
            ]  
        )
    )

    ld.add_action(
        launch_ros.actions.Node(
            namespace="science_manipulator/linear_rail", 
            package='controls_pkg',
            executable='motor',
            parameters=[
                {
                    'arbitration_id': ra_id,
                    'can_network_id' : cn_id
                }
            ]  
        )
    )

    # ld.add_action(
    #     launch_ros.actions.Node(
    #         namespace="science_manipulator/gripper", 
    #         package='controls_pkg',
    #         executable='motor',
    #         parameters=[
    #             {
    #                 'arbitration_id': gr_id,
    #                 'can_network_id' : cn_id
    #             }
    #         ]  
    #     )
    # )
    
    return ld
