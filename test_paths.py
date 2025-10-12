#!/usr/bin/env python3
"""
Test script to verify file path resolution in the AutoSoaring project
"""

import os
import sys

def test_paths():
    print("=== AutoSoaring Path Resolution Test ===")
    print()
    
    # Get the script directory (where this test script is located)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    print(f"Script directory: {script_dir}")
    
    # Test config file path
    config_file = os.path.join(script_dir, "src", "autosoaring_pkg", "config", "thermal_config.yaml")
    print(f"Config file path: {config_file}")
    print(f"Config file exists: {os.path.exists(config_file)}")
    
    if os.path.exists(config_file):
        print("✓ Config file found")
    else:
        print("✗ Config file not found")
        print("Available files in config directory:")
        config_dir = os.path.dirname(config_file)
        if os.path.exists(config_dir):
            for file in os.listdir(config_dir):
                print(f"  - {file}")
        else:
            print(f"  Config directory does not exist: {config_dir}")
    
    print()
    
    # Test plan file path
    plan_file = os.path.join(script_dir, "src", "autosoaring_pkg", "config", "area2.plan")
    print(f"Plan file path: {plan_file}")
    print(f"Plan file exists: {os.path.exists(plan_file)}")
    
    if os.path.exists(plan_file):
        print("✓ Plan file found")
    else:
        print("✗ Plan file not found")
        print("Available files in config directory:")
        config_dir = os.path.dirname(plan_file)
        if os.path.exists(config_dir):
            for file in os.listdir(config_dir):
                print(f"  - {file}")
        else:
            print(f"  Config directory does not exist: {config_dir}")
    
    print()
    
    # Test GZ_Msgs protobuf files
    gz_msgs_dir = os.path.join(script_dir, "src", "autosoaring_pkg", "GZ_Msgs", "python")
    print(f"GZ_Msgs Python directory: {gz_msgs_dir}")
    print(f"GZ_Msgs Python directory exists: {os.path.exists(gz_msgs_dir)}")
    
    if os.path.exists(gz_msgs_dir):
        print("✓ GZ_Msgs Python directory found")
        protobuf_files = [f for f in os.listdir(gz_msgs_dir) if f.endswith('_pb2.py')]
        print(f"Protobuf files found: {protobuf_files}")
    else:
        print("✗ GZ_Msgs Python directory not found")
    
    print()
    
    # Test Python package directory
    pkg_dir = os.path.join(script_dir, "src", "autosoaring_pkg", "autosoaring_pkg")
    print(f"Python package directory: {pkg_dir}")
    print(f"Python package directory exists: {os.path.exists(pkg_dir)}")
    
    if os.path.exists(pkg_dir):
        print("✓ Python package directory found")
        python_files = [f for f in os.listdir(pkg_dir) if f.endswith('.py')]
        print(f"Python files found: {python_files}")
    else:
        print("✗ Python package directory not found")
    
    print()
    print("=== Test Complete ===")

if __name__ == "__main__":
    test_paths()
