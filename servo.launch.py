from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Armer
    arm = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/mavros/cmd/arming',
            'mavros_msgs/srv/CommandBool',
            '{value: true}'
        ],
        output='screen'
    )

    servo_node = Node(
        package='plasmar_servo',
        executable='rc_servo_node',
        output='screen',
        parameters=[
            {'rc_channel': 7},
            {'pwm_spin': 1250},
            {'rate_hz': 20.0},
        ]
    )

    # petit délai pour laisser MAVROS être prêt
    return LaunchDescription([
        arm,
        TimerAction(period=2.0, actions=[servo_node]),
    ])
