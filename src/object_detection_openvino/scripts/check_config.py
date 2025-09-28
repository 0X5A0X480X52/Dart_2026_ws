#!/usr/bin/env python3

"""
配置检查工具，用于验证OpenVINO模型文件和配置是否正确
"""

import os
import sys
import yaml
import argparse


def check_file_exists(file_path, file_type):
    """检查文件是否存在"""
    if not os.path.exists(file_path):
        print(f"❌ {file_type} 文件不存在: {file_path}")
        return False
    
    if not os.path.isfile(file_path):
        print(f"❌ {file_type} 路径不是文件: {file_path}")
        return False
    
    if not os.access(file_path, os.R_OK):
        print(f"❌ {file_type} 文件不可读: {file_path}")
        return False
    
    print(f"✅ {file_type} 文件检查通过: {file_path}")
    return True


def check_model_files(xml_path, bin_path):
    """检查OpenVINO模型文件"""
    print("🔍 检查OpenVINO模型文件...")
    
    xml_ok = check_file_exists(xml_path, "XML模型")
    bin_ok = check_file_exists(bin_path, "BIN权重")
    
    if xml_ok and bin_ok:
        # 检查文件扩展名
        if not xml_path.lower().endswith('.xml'):
            print(f"⚠️  XML文件扩展名不正确: {xml_path}")
        
        if not bin_path.lower().endswith('.bin'):
            print(f"⚠️  BIN文件扩展名不正确: {bin_path}")
        
        # 检查文件大小
        xml_size = os.path.getsize(xml_path)
        bin_size = os.path.getsize(bin_path)
        
        print(f"📊 模型文件大小:")
        print(f"   XML: {xml_size:,} bytes")
        print(f"   BIN: {bin_size:,} bytes")
        
        if xml_size == 0:
            print("❌ XML文件为空")
            return False
        
        if bin_size == 0:
            print("❌ BIN文件为空")
            return False
    
    return xml_ok and bin_ok


def check_config_file(config_path):
    """检查配置文件"""
    print("🔍 检查配置文件...")
    
    if not check_file_exists(config_path, "配置"):
        return False
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        # 检查必需的配置项
        required_keys = [
            'object_detection_openvino_node.ros__parameters.xml_path',
            'object_detection_openvino_node.ros__parameters.bin_path',
            'object_detection_openvino_node.ros__parameters.device'
        ]
        
        node_params = config.get('object_detection_openvino_node', {}).get('ros__parameters', {})
        
        missing_keys = []
        for key in ['xml_path', 'bin_path', 'device', 'input_width', 'input_height']:
            if key not in node_params:
                missing_keys.append(key)
        
        if missing_keys:
            print(f"❌ 配置文件缺少必需参数: {missing_keys}")
            return False
        
        print("✅ 配置文件格式检查通过")
        
        # 显示当前配置
        print("📋 当前配置:")
        for key, value in node_params.items():
            print(f"   {key}: {value}")
        
        return True
        
    except yaml.YAMLError as e:
        print(f"❌ 配置文件YAML格式错误: {e}")
        return False
    except Exception as e:
        print(f"❌ 读取配置文件时发生错误: {e}")
        return False


def check_openvino_device(device):
    """检查OpenVINO设备可用性（需要OpenVINO Python API）"""
    print(f"🔍 检查OpenVINO设备可用性: {device}")
    
    try:
        import openvino.runtime as ov
        core = ov.Core()
        available_devices = core.available_devices
        
        print(f"📱 可用设备: {available_devices}")
        
        if device in available_devices:
            print(f"✅ 设备 '{device}' 可用")
            return True
        else:
            print(f"❌ 设备 '{device}' 不可用")
            return False
            
    except ImportError:
        print("⚠️  OpenVINO Python API未安装，跳过设备检查")
        return True
    except Exception as e:
        print(f"❌ 检查设备时发生错误: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description='OpenVINO目标检测节点配置检查工具')
    parser.add_argument('--config', '-c', 
                       default='config/params.yaml',
                       help='配置文件路径 (默认: config/params.yaml)')
    parser.add_argument('--xml', 
                       help='XML模型文件路径 (覆盖配置文件中的路径)')
    parser.add_argument('--bin',
                       help='BIN权重文件路径 (覆盖配置文件中的路径)')
    
    args = parser.parse_args()
    
    print("🚀 OpenVINO目标检测节点配置检查工具")
    print("=" * 50)
    
    # 检查配置文件
    config_ok = check_config_file(args.config)
    
    if not config_ok:
        print("\n❌ 配置文件检查失败")
        sys.exit(1)
    
    # 读取配置以获取模型路径
    try:
        with open(args.config, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        node_params = config['object_detection_openvino_node']['ros__parameters']
        xml_path = args.xml or node_params['xml_path']
        bin_path = args.bin or node_params['bin_path']
        device = node_params['device']
        
    except Exception as e:
        print(f"❌ 读取配置失败: {e}")
        sys.exit(1)
    
    print()
    
    # 检查模型文件
    model_ok = check_model_files(xml_path, bin_path)
    
    print()
    
    # 检查设备
    device_ok = check_openvino_device(device)
    
    print()
    print("=" * 50)
    
    if config_ok and model_ok and device_ok:
        print("🎉 所有检查通过！节点应该能够正常运行。")
        sys.exit(0)
    else:
        print("❌ 存在配置问题，请修复后重试。")
        sys.exit(1)


if __name__ == '__main__':
    main()