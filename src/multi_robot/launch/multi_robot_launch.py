
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('multi_robot')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/multi_robots.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions for robot_1
    
    approach_items_target_1_cmd = Node(
        package='multi_robot',
        executable='approach_items_target_node',
        name='approach_items_target_1',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_1","red_balls"]
          }
        ])

    drop_items_1_cmd = Node(
        package='multi_robot',
        executable='drop_items_node',
        name='drop_items_1',
        namespace=namespace,
        output='screen',
        parameters=[
         # example_dir + '/config/params_multi_robot.yaml',
          {
            'specialized_arguments': ["p6building","robot_1","red_balls"]
            #'action_name': 'drop_items',
            #'enable_groot_monitoring' : True,
            #'publisher_port': 1668,
            #'server_port': 1669,
            #'bt_xml_file': example_dir + '/behavior_trees_xml/drop.xml'
          }
        ])

    grab_items_1_cmd = Node(
        package='multi_robot',
        executable='grab_items_node',
        name='grab_items_1',
        namespace=namespace,
        output='screen',
        parameters=[
         #example_dir + '/config/params_multi_robot.yaml',
          {
            'specialized_arguments': ["p6building","robot_1","red_balls"]
            #'action_name': 'grab_items',
            #'enable_groot_monitoring' : True,
            #'publisher_port': 1678,
            #'server_port': 1679,
            #'bt_xml_file': example_dir + '/behavior_trees_xml/grab.xml'
          }
        ])

    handle_items_1_cmd = Node(
        package='multi_robot',
        executable='handle_items_node',
        name='handle_items_1',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_1","red_balls"]
          }
        ])

    approach_items_1_cmd = Node(
        package='multi_robot',
        executable='approach_items_node',
        name='approach_items_1',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_1","red_balls"]
          }
        ])

    # Specify the actions for robot_2
    
    approach_items_target_2_cmd = Node(
        package='multi_robot',
        executable='approach_items_target_node',
        name='approach_items_target_2',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_2","blue_balls"]
          }
        ])

    drop_items_2_cmd = Node(
        package='multi_robot',
        executable='drop_items_node',
        name='drop_items_2',
        namespace=namespace,
        output='screen',
        parameters=[
          #example_dir + '/config/params_multi_robot.yaml',
          {
            'specialized_arguments': ["p6building","robot_2","blue_balls"]
            #'action_name': 'drop_items',
            #'enable_groot_monitoring' : True,
            #'publisher_port': 1670,
            #'server_port': 1671,
            #'bt_xml_file': example_dir + '/behavior_trees_xml/drop.xml'
          }
        ])

    grab_items_2_cmd = Node(
        package='multi_robot',
        executable='grab_items_node',
        name='grab_items_2',
        namespace=namespace,
        output='screen',
        parameters=[
         # example_dir + '/config/params_multi_robot.yaml',
          {
            'specialized_arguments': ["p6building","robot_2","blue_balls"]
            #'action_name': 'grab_items',
            #'enable_groot_monitoring' : True,
            #'publisher_port': 1674,
            #'server_port': 1675,
            #'bt_xml_file': example_dir + '/behavior_trees_xml/grab.xml'
          }
        ])

    handle_items_2_cmd = Node(
        package='multi_robot',
        executable='handle_items_node',
        name='handle_items_2',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_2","blue_balls"]
          }
        ])

    approach_items_2_cmd = Node(
        package='multi_robot',
        executable='approach_items_node',
        name='approach_items_2',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_2","blue_balls"]
          }
        ])

    # Specify the actions for robot_3
    
    approach_items_target_3_cmd = Node(
        package='multi_robot',
        executable='approach_items_target_node',
        name='approach_items_target_3',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_3","white_boxes"]
          }
        ])

    drop_items_3_cmd = Node(
        package='multi_robot',
        executable='drop_items_node',
        name='drop_items_3',
        namespace=namespace,
        output='screen',
        parameters=[
          #example_dir + '/config/params_multi_robot.yaml',
          {
            'specialized_arguments': ["p6building","robot_3","white_boxes"]
            #'action_name': 'drop_items',
            #'enable_groot_monitoring' : True,
            #'publisher_port': 1672,
            #'server_port': 1673,
            #'bt_xml_file': example_dir + '/behavior_trees_xml/drop.xml'
          }
        ])

    grab_items_3_cmd = Node(
        package='multi_robot',
        executable='grab_items_node',
        name='grab_items_3',
        namespace=namespace,
        output='screen',
        parameters=[
          #example_dir + '/config/params_multi_robot.yaml',
          {
            'specialized_arguments': ["p6building","robot_3","white_boxes"]
            #'action_name': 'grab_items',
           # 'enable_groot_monitoring' : True,
            #'publisher_port': 1676,
            #'server_port': 1677,
            #'bt_xml_file': example_dir + '/behavior_trees_xml/grab.xml'
          }
        ])

    handle_items_3_cmd = Node(
        package='multi_robot',
        executable='handle_items_node',
        name='handle_items_3',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_3","white_boxes"]
          }
        ])

    approach_items_3_cmd = Node(
        package='multi_robot',
        executable='approach_items_node',
        name='approach_items_3',
        namespace=namespace,
        output='screen',
        parameters=[
          {
            'specialized_arguments': ["p6building","robot_3","white_boxes"]
          }
        ])

    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    #robot_1
    ld.add_action(approach_items_target_1_cmd)
    ld.add_action(drop_items_1_cmd)
    ld.add_action(grab_items_1_cmd)
    ld.add_action(handle_items_1_cmd)
    ld.add_action(approach_items_1_cmd)

    #robot_2
    ld.add_action(approach_items_target_2_cmd)
    ld.add_action(drop_items_2_cmd)
    ld.add_action(grab_items_2_cmd)
    ld.add_action(handle_items_2_cmd)
    ld.add_action(approach_items_2_cmd)

    #robot_3
    ld.add_action(approach_items_target_3_cmd)
    ld.add_action(drop_items_3_cmd)
    ld.add_action(grab_items_3_cmd)
    ld.add_action(handle_items_3_cmd)
    ld.add_action(approach_items_3_cmd)


    return ld
