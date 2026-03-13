import os
import pathlib

import yaml
from easy_handeye2_msgs.msg import HandeyeCalibration, HandeyeCalibrationParameters
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rosidl_runtime_py import set_message_fields, message_to_yaml

from . import CALIBRATIONS_DIRECTORY


def filepath_for_calibration(name) -> pathlib.Path:
    return CALIBRATIONS_DIRECTORY / f'{name}.calib'


class HandeyeCalibrationParametersProvider:
    def __init__(self, node: Node):
        self.node = node
        # declare and read parameters
        self.node.declare_parameter('name', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('calibration_type', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('robot_base_frame', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('robot_effector_frame', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('tracking_base_frame', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('tracking_marker_frame', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.node.declare_parameter('freehand_robot_movement', True)

    def read(self):
        ret = HandeyeCalibrationParameters(
            name=self.node.get_parameter('name').get_parameter_value().string_value,
            calibration_type=self.node.get_parameter('calibration_type').get_parameter_value().string_value,
            robot_base_frame=self.node.get_parameter('robot_base_frame').get_parameter_value().string_value,
            robot_effector_frame=self.node.get_parameter('robot_effector_frame').get_parameter_value().string_value,
            tracking_base_frame=self.node.get_parameter('tracking_base_frame').get_parameter_value().string_value,
            tracking_marker_frame=self.node.get_parameter('tracking_marker_frame').get_parameter_value().string_value,
            freehand_robot_movement=self.node.get_parameter('freehand_robot_movement').get_parameter_value().bool_value,
        )
        return ret


def _normalize_legacy_calibration(data: dict, name: str) -> dict:
    if not isinstance(data, dict):
        raise ValueError(f'Calibration "{name}" must be a mapping, got {type(data).__name__}')

    if 'parameters' in data and 'transform' in data:
        parameters = dict(data['parameters'] or {})
        transform = dict(data['transform'] or {})
    else:
        parameters = {}
        transform = {}
        for key, value in data.items():
            if key in {'transform', 'translation', 'rotation'}:
                continue
            parameters[key] = value

        if 'transform' in data:
            transform = dict(data['transform'] or {})
        else:
            translation = dict(data.get('translation') or {})
            rotation = dict(data.get('rotation') or {})
            if translation or rotation:
                transform = {'translation': translation, 'rotation': rotation}

    legacy_eye_on_hand = parameters.pop('eye_on_hand', None)
    if 'calibration_type' not in parameters and legacy_eye_on_hand is not None:
        parameters['calibration_type'] = 'eye_in_hand' if legacy_eye_on_hand else 'eye_on_base'

    return {
        'parameters': parameters,
        'transform': transform,
    }


def load_calibration(name, calibration_file=None) -> HandeyeCalibration:
    filepath = pathlib.Path(calibration_file) if calibration_file else filepath_for_calibration(name)
    with open(filepath) as f:
        m = yaml.full_load(f.read())
    m = _normalize_legacy_calibration(m, name)
    ret = HandeyeCalibration()
    set_message_fields(ret, m)
    return ret


def save_calibration(calibration: HandeyeCalibration) -> pathlib.Path:
    if not os.path.exists(CALIBRATIONS_DIRECTORY):
        os.makedirs(CALIBRATIONS_DIRECTORY)
    filepath = filepath_for_calibration(calibration.parameters.name)
    with open(filepath, 'w') as f:
        f.write(message_to_yaml(calibration))
    return filepath
