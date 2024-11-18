#!/usr/bin/env python3

import argparse
import sys
import os
import yaml
from pathlib import Path
import itertools
from typing import List, Dict, Set, Tuple


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Generate UPPAAL model code based on configuration file'
    )
    
    parser.add_argument(
        '--config_file',
        type=str,
        required=True,
        help='Path to the configuration YAML file'
    )
    
    parser.add_argument(
        '--output_dir',
        type=str,
        required=True,
        help='Directory path where the output files will be saved'
    )
    
    return parser.parse_args()


def validate_inputs(config_file: str, output_dir: str) -> bool:
    """
    Validate the input arguments.
    
    Args:
        config_file (str): Path to the configuration file
        output_dir (str): Path to the output directory
    
    Returns:
        bool: True if inputs are valid, False otherwise
    """
    # Check if config file exists and is a YAML file
    if not os.path.isfile(config_file):
        print(f"Error: Config file '{config_file}' does not exist")
        return False
    
    if not config_file.endswith(('.yml', '.yaml')):
        print(f"Error: Config file '{config_file}' is not a YAML file")
        return False
    
    # Check if output directory exists, if not create it
    output_path = Path(output_dir)
    try:
        output_path.mkdir(parents=True, exist_ok=True)
    except Exception as e:
        print(f"Error: Could not create output directory '{output_dir}': {str(e)}")
        return False
    
    return True


def read_config(config_file: str) -> dict:
    """
    Read and parse the YAML configuration file.
    
    Args:
        config_file (str): Path to the configuration file
    
    Returns:
        dict: Parsed configuration data
    """
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"Error: Failed to read config file: {str(e)}")
        sys.exit(1)


def generate_declarations_file(config: dict, output_dir: str):
    """
    Generate the declarations C file containing constants and tables.
    
    Args:
        config (dict): Parsed configuration data
        output_dir (str): Output directory path
    """
    node_name = config['node']['name']
    callbacks = config['node']['callbacks']
    callback_groups = config['node']['callback_groups']
    threads = config['node']['executor']['threads']
    
    # Create a mapping of callback group names to their indices
    group_name_to_idx = {group['name']: idx for idx, group in enumerate(callback_groups)}
    
    # Create the callback group table
    cb_group_table = [group_name_to_idx[cb['callback_group']] for cb in callbacks]
    
    # Create the callback type table
    cb_type_table = [cb['type'] for cb in callbacks]
    
    # Generate the content
    content = [
        "// User parameters",
        f"const int NUM_CB = {len(callbacks)};    // user config param",
        f"const int NUM_CB_GROUP = {len(callback_groups)};    // user config param",
        f"const int NUM_THREAD = {len(threads)};    // user config param",
        "",
        "const int MAX_CYCLE = 1;",
        "",
        "typedef int[0, NUM_CB-1] CbId;",
        "typedef int[0, NUM_CB_GROUP-1] GroupId;",
        "typedef int[0, NUM_THREAD-1] ThreadId;",
        "",
        "typedef int[0, 3] CbType;",
        "const CbType TMR = 0;",
        "const CbType SUB = 1;",
        "const CbType SRV = 2;",
        "const CbType CLT = 3;",
        "",
        "typedef int[0, 1] GroupType;",
        "const GroupType REENTRANT = 0;",
        "const GroupType EXCLUSIVE = 1;",
        "",
        f"const GroupId cb_group_table[NUM_CB] = {{{', '.join(map(str, cb_group_table))}}};    // user config param",
        f"const CbType cb_type_table[NUM_CB] = {{{', '.join(cb_type_table)}}};    // user config param",
        "",
        "// Callback group",
        "bool group_closed[NUM_CB_GROUP];",
        "chan group_close[NUM_CB_GROUP];",
        "chan group_open[NUM_CB_GROUP];",
        "",
        "// Callback",
        "bool ready_set[NUM_CB];",
        "",
        "// Thread",
        "chan cb_assign_signal[NUM_THREAD];",
        "broadcast chan thread_free;",
        "// int[0, NUM_THREAD] num_running_thread;",
        "",
        "bool thread_running[NUM_THREAD];",
        "CbId thread_cb_assign_table[NUM_THREAD];",
        "",
        "bool cb_running(CbId cb_id){",
        "    int t_id;",
        "    for (t_id = 0; t_id < NUM_THREAD; t_id++){",
        "        if (thread_running[t_id]){",
        "            if (thread_cb_assign_table[t_id] == cb_id){",
        "                return true;",
        "            }",
        "        }",
        "    }",
        "    return false;",
        "}",
        "",
        "CbId get_thread_running_cb(ThreadId t_id){",
        "    return thread_cb_assign_table[t_id];",
        "}",
        "",
        "GroupId get_thread_running_group(ThreadId t_id){",
        "    return cb_group_table[thread_cb_assign_table[t_id]];",
        "}"
    ]
    
    # Write to file
    output_file = os.path.join(output_dir, f"{node_name}_declarations.c")
    with open(output_file, 'w') as f:
        f.write('\n'.join(content))


def generate_system_definition_file(config: dict, output_dir: str):
    """
    Generate the system C file containing callback and thread definition.
    
    Args:
        config (dict): Parsed configuration data
        output_dir (str): Output directory path
    """
    node_name = config['node']['name']
    callbacks = config['node']['callbacks']
    callback_groups = config['node']['callback_groups']
    executor = config['node']['executor']
    threads = executor['threads']
    
    # Generate callback declarations
    cb_decls = [f"{cb['name']} = Callback({idx});" for idx, cb in enumerate(callbacks)]
    
    # Generate callback group declarations
    group_decls = [f"{group['name']} = CallbackGroup({group['type']}, {idx});" 
                  for idx, group in enumerate(callback_groups)]
    
    # Generate thread declarations
    thread_decls = [f"{thread['name']} = Thread({idx});" for idx, thread in enumerate(threads)]
    
    # Generate system components list
    system_components = (
        [executor['name']] +
        [thread['name'] for thread in threads] +
        [group['name'] for group in callback_groups] +
        [cb['name'] for cb in callbacks]
    )
    
    # Generate the content
    content = [
        f"// System name: {node_name}",
        "",
        "// Callbacks",
        '\n'.join(cb_decls),
        "",
        "// Callback group",
        '\n'.join(group_decls),
        "",
        "// Executor and Threads",
        f"{executor['name']} = Executor();",
        '\n'.join(thread_decls),
        "",
        "// Initialize the system",
        "system",
        ',\n'.join(system_components) + ";"
    ]
    
    # Write to file
    output_file = os.path.join(output_dir, f"{node_name}_system.c")
    with open(output_file, 'w') as f:
        f.write('\n'.join(content))


def generate_code(config: dict, output_dir: str):
    """
    Generate UPPAAL model code based on the configuration.
    
    Args:
        config (dict): Parsed configuration data
        output_dir (str): Output directory path
    """
    try:
        # Generate declarations file
        generate_declarations_file(config, output_dir)
        
        # Generate system file
        generate_system_definition_file(config, output_dir)

        generate_properties_file(config, output_dir)
        
        print(f"Successfully generated code files in {output_dir}")
        
    except Exception as e:
        print(f"Error generating code: {str(e)}")
        sys.exit(1)


def validate_config(config: dict) -> bool:
    """
    Validate the YAML configuration structure and content.
    
    Args:
        config (dict): Parsed configuration data
    
    Returns:
        bool: True if configuration is valid, False otherwise
    """
    # Check if node exists and is a dict
    if 'node' not in config or not isinstance(config['node'], dict):
        print("Error: Configuration must have a 'node' as the top level element")
        return False
    
    node = config['node']
    
    # Validate node has required fields
    required_node_fields = {'name', 'callbacks', 'callback_groups', 'executor'}
    missing_fields = required_node_fields - set(node.keys())
    if missing_fields:
        print(f"Error: Node is missing required fields: {missing_fields}")
        return False
    
    # Validate node name
    if not isinstance(node['name'], str) or not node['name']:
        print("Error: Node name must be a non-empty string")
        return False
    
    # Track all names to check for duplicate
    used_names = {node['name']}

    # Validate callbacks
    if not isinstance(node['callbacks'], list):
        print("Error: 'callbacks' must be a list")
        return False
    
    if len(node['callbacks']) < 1:
        print("Error: Must have at least one callback")
        return False
    
    valid_cb_types = {'TMR', 'SUB', 'SRV', 'CLT'}
    callback_group_names = {group['name'] for group in node['callback_groups']}
    
    for idx, callback in enumerate(node['callbacks']):
        if not isinstance(callback, dict):
            print(f"Error: Callback {idx} must be a dictionary")
            return False
        
        # Check required callback fields
        required_cb_fields = {'name', 'type', 'callback_group'}
        missing_fields = required_cb_fields - set(callback.keys())
        if missing_fields:
            print(f"Error: Callback {callback.get('name', idx)} is missing required fields: {missing_fields}")
            return False
        
        # Validate callback name
        if not isinstance(callback['name'], str) or not callback['name']:
            print(f"Error: Callback {idx} must have a non-empty string name")
            return False
        
        # Check for duplicate names
        if callback['name'] in used_names:
            print(f"Error: Duplicate name '{callback['name']}' found")
            return False
        used_names.add(callback['name'])
        
        # Validate callback type
        if callback['type'] not in valid_cb_types:
            print(f"Error: Callback {callback['name']} has invalid type. Must be one of {valid_cb_types}")
            return False
        
        # Validate callback group reference
        if callback['callback_group'] not in callback_group_names:
            print(f"Error: Callback {callback['name']} references non-existent callback group '{callback['callback_group']}'")
            return False
    
    # Validate callback groups
    if not isinstance(node['callback_groups'], list):
        print("Error: 'callback_groups' must be a list")
        return False
    
    if len(node['callback_groups']) < 1:
        print("Error: Must have at least one callback group")
        return False
    
    valid_group_types = {'REENTRANT', 'EXCLUSIVE'}
    
    for idx, group in enumerate(node['callback_groups']):
        if not isinstance(group, dict):
            print(f"Error: Callback group {idx} must be a dictionary")
            return False
        
        # Check required group fields
        required_group_fields = {'name', 'type'}
        missing_fields = required_group_fields - set(group.keys())
        if missing_fields:
            print(f"Error: Callback group {group.get('name', idx)} is missing required fields: {missing_fields}")
            return False
        
        # Validate group name
        if not isinstance(group['name'], str) or not group['name']:
            print(f"Error: Callback group {idx} must have a non-empty string name")
            return False
        
        # Check for duplicate names
        if group['name'] in used_names:
            print(f"Error: Duplicate name '{group['name']}' found")
            return False
        used_names.add(group['name'])
        
        # Validate group type
        if group['type'] not in valid_group_types:
            print(f"Error: Callback group {group['name']} has invalid type. Must be one of {valid_group_types}")
            return False
    
    # Validate executor
    executor = node['executor']
    if not isinstance(executor, dict):
        print("Error: 'executor' must be a dictionary")
        return False
    
    # Check required executor fields
    required_exec_fields = {'name', 'threads'}
    missing_fields = required_exec_fields - set(executor.keys())
    if missing_fields:
        print(f"Error: Executor is missing required fields: {missing_fields}")
        return False
    
    # Validate executor name
    if not isinstance(executor['name'], str) or not executor['name']:
        print("Error: Executor must have a non-empty string name")
        return False
    
    # Check for duplicate names
    if executor['name'] in used_names:
        print(f"Error: Duplicate name '{executor['name']}' found")
        return False
    used_names.add(executor['name'])
    
    # Validate threads
    if not isinstance(executor['threads'], list):
        print("Error: 'threads' must be a list")
        return False
    
    if len(executor['threads']) < 1:
        print("Error: Must have at least one thread")
        return False
    
    for idx, thread in enumerate(executor['threads']):
        if not isinstance(thread, dict):
            print(f"Error: Thread {idx} must be a dictionary")
            return False
        
        # Validate thread name
        if 'name' not in thread:
            print(f"Error: Thread {idx} is missing required field 'name'")
            return False
        
        if not isinstance(thread['name'], str) or not thread['name']:
            print(f"Error: Thread {idx} must have a non-empty string name")
            return False
        
        # Check for duplicate names
        if thread['name'] in used_names:
            print(f"Error: Duplicate name '{thread['name']}' found")
            return False
        used_names.add(thread['name'])
    
    return True


def generate_properties_file(config: dict, output_dir: str):
    """
    Generate property file from templates based on the configuration.
    
    Args:
        config (dict): Parsed configuration data
        output_file (str): Output file path for properties
    """
    # Extract node configuration for easier access
    node_config = config['node']
    node_name = node_config['name']
    
    # Create ID mappings
    callback_ids = {cb['name']: idx for idx, cb in enumerate(node_config['callbacks'])}
    group_ids = {group['name']: idx for idx, group in enumerate(node_config['callback_groups'])}
    
    # Get executor name
    executor_name = node_config['executor']['name']
    
    # List to store all properties
    properties = []
    
    # Template 0: A[] not deadlock
    prop = f"/*\nAbsence of node deadlock\n*/\nA[] not deadlock\n"
    properties.append(prop)

    # Template 1: E<> cb_running($callback_id)
    for callback_id in callback_ids.values():
        prop = f"/*\nCallback function executability\n*/\nE<> cb_running({callback_id})\n"
        properties.append(prop)
    
    # Template 2: A[] not (cb_running($callback_id_1) and cb_running($callback_id_2))
    # Get all possible combinations of two different callback IDs
    callback_pairs = list(itertools.combinations(callback_ids.values(), 2))
    for id1, id2 in callback_pairs:
        prop = f"/*\nMutually exclusive execution of callback functions\n*/\nA[] not (cb_running({id1}) and cb_running({id2}))\n"
        properties.append(prop)
    
    # Template 3: A[] cb_running($callback_id) imply group_closed[cb_group_table[$callback_id]]
    for callback_name, callback_id in callback_ids.items():
        # Find the callback's group
        callback_info = next(cb for cb in node_config['callbacks'] if cb['name'] == callback_name)
        group_id = group_ids[callback_info['callback_group']]
        prop = f"/*\nBlocking multi-thread access to the callback group (mutual exclusion)\n*/\nA[] cb_running({callback_id}) imply group_closed[cb_group_table[{callback_id}]]\n"
        properties.append(prop)
    
    # Template 4: E<> (cb_running($callback_id_1) and cb_running($callback_id_2))
    for id1, id2 in callback_pairs:
        prop = f"/*\nConcurrent execution of callback functions\n*/\nE<> (cb_running({id1}) and cb_running({id2}))\n"
        properties.append(prop)
    
    # Template 5: A[] not group_closed($callback_group_id)
    for group_id in group_ids.values():
        prop = f"/*\nAllowing multi-thread access to the callback group (reentrant)\n*/\nA[] not group_closed[{group_id}]\n"
        properties.append(prop)
    
    # Template 6: ($executor_name.schedule_start and $executor_name.ready_set_copy[$callback_id] 
    # and !cb_running($callback_id)) --> cb_running($callback_id)
    for callback_id in callback_ids.values():
        prop = (f"/*\nAbsence of callback function starvation\n*/\n({executor_name}.schedule_start and {executor_name}.ready_set_copy[{callback_id}] "
               f"and !cb_running({callback_id})) --> cb_running({callback_id})\n")
        properties.append(prop)
    
    # Write properties to file
    output_file = os.path.join(output_dir, f"{node_name}_query.q")
    with open(output_file, 'w') as f:
        f.write('\n'.join(properties))


def main():
    """Main function."""
    # Parse command line arguments
    args = parse_arguments()
    
    # Validate inputs
    if not validate_inputs(args.config_file, args.output_dir):
        sys.exit(1)
    
    # Read configuration
    config = read_config(args.config_file)


    # Validate configuration structure
    if not validate_config(config):
        sys.exit(1)
    
    # Generate code
    generate_code(config, args.output_dir)


if __name__ == "__main__":
    main()