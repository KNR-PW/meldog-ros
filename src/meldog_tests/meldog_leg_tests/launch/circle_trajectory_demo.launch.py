from launch import LaunchDescription
from launch_ros.actions import Node



LENGTH_1 = 0.225                    # Length of body one [m]
LENGTH_2 = 0.225                    # Length of body two [m]
START_POSITION = [0.0, -0.30]       # Start position (-y axis) [m]
RADIUS = 0.1                        # Radius of circle [m]
ANGULAR_VELOCITY = 3.14*2           # Angular velocity of motion [rad/s]
def generate_launch_description():
    inverse_kinematics = Node(
        package = 'meldog_leg_tests',
        executable = 'inverse_kinematics_2D',
        parameters = [{
            'length_1': LENGTH_1,
            'length_2': LENGTH_2,
            'start_position': START_POSITION
        }],
        output = 'screen'
    )
    forward_kinematics = Node(
        package = 'meldog_leg_tests',
        executable = 'forward_kinematics_2D',
        parameters = [{
            'length_1': LENGTH_1,
            'length_2': LENGTH_2
        }],
        output = 'screen'
    )
    linear_trajectory = Node(
        package = 'meldog_leg_tests',
        executable = 'circle_trajectory',
        parameters = [{
            'start_position': START_POSITION,
            'radius': RADIUS,
            'angular_velocity': ANGULAR_VELOCITY
        }],
        output = 'screen'
    )

    nodes = [inverse_kinematics, forward_kinematics, linear_trajectory]
    ld = LaunchDescription(nodes)
    return ld
