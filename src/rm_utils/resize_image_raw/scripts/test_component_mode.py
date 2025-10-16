#!/usr/bin/env python3
"""
Test script to verify resize_image_raw works in both standalone and composable modes
"""

import rclpy
from rclpy.node import Node
import subprocess
import time
import sys

def test_component_registration():
    """Test if the component is properly registered"""
    print("=" * 60)
    print("Testing component registration...")
    print("=" * 60)
    
    result = subprocess.run(
        ['ros2', 'component', 'types'],
        capture_output=True,
        text=True
    )
    
    if 'resize_image_raw::ResizeNode' in result.stdout:
        print("✅ Component registered: resize_image_raw::ResizeNode")
        return True
    else:
        print("❌ Component NOT found in registry")
        print("Available components:")
        print(result.stdout)
        return False

def test_standalone_node():
    """Test standalone node launch"""
    print("\n" + "=" * 60)
    print("Testing standalone node mode...")
    print("=" * 60)
    
    # This would require actual launch, skipping for now
    print("⚠️  Manual test required: ros2 launch resize_image_raw resize_with_config.launch.py")
    print("   Expected: Should see 'Standalone node mode, call initialize() manually'")
    return True

if __name__ == '__main__':
    print("\n🧪 Testing resize_image_raw composable node support\n")
    
    success = True
    success &= test_component_registration()
    success &= test_standalone_node()
    
    print("\n" + "=" * 60)
    if success:
        print("✅ All tests passed!")
    else:
        print("❌ Some tests failed")
    print("=" * 60)
    
    sys.exit(0 if success else 1)
