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

    # petit délai pour laisser MAVROS être prêt
    mixer_node = Node(
        package='plasmar_servo',
        executable='rc_manipuler_robot',
        output='screen',
        parameters=[
            
            {'ch_rear_left': 1},
            {'ch_rear_right': 2},
            {'ch_front_left': 3},
            {'ch_front_right': 4},
            {'ch_servo': 7},

            {'thr_pwm_center': 1500},
            {'thr_pwm_scale': 400.0},
            {'thr_pwm_min': 1100},
            {'thr_pwm_max': 1900},

            {'servo_pwm_center': 1500},
            {'servo_pwm_up': 1700},
            {'servo_pwm_down': 1300},

            {'deadband': 0.05},
            {'rate_hz': 20.0},
        ]
    )

    return LaunchDescription([
        arm,
        TimerAction(period=2.0, actions=[mixer_node]),
    ])