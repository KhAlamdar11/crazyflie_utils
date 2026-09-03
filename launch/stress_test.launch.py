"""Launch one crazyflie_utils stress test.

    ros2 launch crazyflie_utils stress_test.launch.py scenario:=sync_vs_async
    ros2 launch crazyflie_utils stress_test.launch.py \\
        scenario:=takeoff_land_cycle num_uavs:=4 dry_run:=true

``set`` takes a semicolon-separated list of dotted overrides, mirroring the
repeatable ``--set`` flag of the node::

    set:="test.params.cycles=25;test.params.takeoff_height=0.8"

Everything else stays in the YAML files under ``config/``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'scenario', default_value='service_ping',
        description='Config file in config/ (without .yaml) or scenario name'),
    DeclareLaunchArgument(
        'common', default_value='common.yaml',
        description='Shared defaults file merged before the scenario'),
    DeclareLaunchArgument(
        'num_uavs', default_value='',
        description='Override fleet.num_uavs (empty: use the config)'),
    DeclareLaunchArgument(
        'namespaces', default_value='',
        description='Comma-separated drone namespaces, e.g. cf1,cf3'),
    DeclareLaunchArgument(
        'dry_run', default_value='false',
        description='Log commands instead of sending them'),
    DeclareLaunchArgument(
        'tag', default_value='',
        description='Suffix for the report file names'),
    DeclareLaunchArgument(
        'output_dir', default_value='',
        description='Directory for JSON/CSV artefacts'),
    DeclareLaunchArgument(
        'set', default_value='',
        description='Semicolon-separated key.path=value overrides'),
]


def _launch_setup(context, *args, **kwargs):
    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context).strip()

    arguments = [value('scenario'), '--common', value('common')]

    if value('num_uavs'):
        arguments += ['--num-uavs', value('num_uavs')]
    if value('namespaces'):
        arguments += ['--namespaces', value('namespaces')]
    if value('dry_run').lower() in ('true', '1', 'yes'):
        arguments += ['--dry-run']
    if value('tag'):
        arguments += ['--tag', value('tag')]
    if value('output_dir'):
        arguments += ['--output-dir', value('output_dir')]
    for override in value('set').split(';'):
        if override.strip():
            arguments += ['--set', override.strip()]

    return [Node(
        package='crazyflie_utils',
        executable='stress_test',
        name='crazyflie_stress_test',
        output='screen',
        emulate_tty=True,
        arguments=arguments,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=_launch_setup)])
