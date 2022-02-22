# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('controller')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file':
                example_dir + '/pddl_1/domain_1.pddl:' +
                example_dir + '/pddl_2/domain_2.pddl:' +
                example_dir + '/pddl_3/domain_objHandling.pddl'
                #example_dir + '/pddl_4/ObjHandling_Domain.pddl'
               
        }.items())


    # Specify the actions
    move_cmd = Node(
        package='controller',
        executable ='move_action_node',
        name ='move_action_node',
        namespace = 'move_action_node',
        output ='screen',
        parameters=[])

    charge_cmd = Node(
        package ='controller',
        executable ='charge_action_node',
        name ='charge_action_node',
        namespace = 'charge_action_node',
        output='screen',
        parameters=[])

    ask_charge_cmd = Node(
        package = 'controller',
        executable = 'ask_charge_action_node',
        name = 'ask_charge_action_node',
        namespace = 'ask_charge_action_node',
        output = 'screen',
        parameters = [])  
        
    approach_balls_cmd = Node(
        package = 'controller',
        executable = 'approach_balls_action_node',
        name = 'approach_balls_action_node',
        namespace = 'approach_balls_action_node',
        output = 'screen',
        parameters = []) 

    approach_balls_target_cmd = Node(
        package = 'controller',
        executable = 'approach_balls_target_action_node',
        name = 'approach_balls_target_action_node',
        namespace = 'approach_balls_target_action_node',
        output = 'screen',
        parameters = [])  
        
        
    drop_balls_cmd = Node(
        package = 'controller',
        executable = 'drop_balls_action_node',
        name = 'drop_balls_action_node',
        namespace = 'drop_balls_action_node',
        output = 'screen',
        parameters = [])  
        
    explore_cmd = Node(
        package = 'controller',
        executable = 'explore_action_node',
        name = 'explore_action_node',
        namespace = 'explore_action_node',
        output = 'screen',
        parameters = []) 
        
    grab_balls_cmd = Node(
        package = 'controller',
        executable = 'grab_balls_action_node',
        name = 'grab_balls_action_node',
        namespace = 'grab_balls_action_node',
        output = 'screen',
        parameters = [])  

    handle_balls_cmd = Node(
        package = 'controller',
        executable = 'handle_balls_action_node',
        name = 'handle_balls_action_node',
        namespace = 'handle_balls_action_node',
        output = 'screen',
        parameters = [])
        
        
    map_cmd = Node(
        package = 'controller',
        executable = 'map_action_node',
        name = 'map_action_node',
        namespace = 'map_action_node',
        output = 'screen',
        parameters = [])
        
    wander_cmd = Node(
        package = 'controller',
        executable = 'wander_action_node',
        name = 'wander_action_node',
        namespace = 'wander_action_node',
        output = 'screen',
        parameters = [])  
        
         # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
   # ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(charge_cmd)
    ld.add_action(ask_charge_cmd)

    ld.add_action(approach_balls_cmd)
    ld.add_action(approach_balls_target_cmd)
    ld.add_action(drop_balls_cmd)

    ld.add_action(explore_cmd)
    ld.add_action(grab_balls_cmd)
    ld.add_action(handle_balls_cmd)

    ld.add_action(map_cmd)
    ld.add_action(wander_cmd)


   

    return ld
