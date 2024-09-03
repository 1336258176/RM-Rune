import launch
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# ------------------------------------------ rune config ----------------------------------------- #
rune_recognition_config = os.path.join(
    get_package_share_directory('rune'),
    'config',
    'reco_config.yaml'
)
rune_hometest_reco_config = os.path.join(
    get_package_share_directory('rune'),
    'config',
    'reco_home_test.yaml'
)

def generate_launch_description():
    rune_container = ComposableNodeContainer(
            name='rune_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     package='rune',
                #     plugin='rune::TransformNode',
                #     name='transform_node',
                #     namespace='rune',
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),

                ComposableNode(
                    package='rune',
                    plugin='rune::RecognitionNode',
                    name='recognition_node',
                    namespace='rune',
                    parameters=[rune_recognition_config],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),

                # ComposableNode(
                #     package='rune',
                #     plugin='rune::PredictionNode',
                #     name='prediction_node',
                #     namespace='rune',
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),

                ComposableNode(
                    package='rune',
                    plugin='rune::TestNode',
                    name='test_node',
                    namespace='rune',
                    extra_arguments=[{'use_intra_process_comms': True}]

                )
            ],
            output='both',
    )

    return launch.LaunchDescription([rune_container])