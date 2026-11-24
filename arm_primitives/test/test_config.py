"""Check that every shipped arm config loads, and that bad configs are rejected."""

from pathlib import Path

import pytest
import yaml

from arm_primitives.config import ConfigError, parse_arm_config

ARMS = Path(__file__).resolve().parent.parent / 'config' / 'arms'


def load(name):
    return yaml.safe_load((ARMS / f'{name}.yaml').read_text())


@pytest.mark.parametrize('path', sorted(ARMS.glob('*.yaml')), ids=lambda p: p.stem)
def test_shipped_configs_load(path):
    config = parse_arm_config(yaml.safe_load(path.read_text()))
    assert config.name == path.stem
    assert config.joints
    assert {'mock', 'real'} <= set(config.bringup)
    for pose in config.named_poses.values():
        assert set(pose) <= set(config.joints)


def test_fer_uses_franka_gripper():
    config = parse_arm_config(load('fer'))
    assert config.gripper.type == 'franka'
    assert config.planning_group == 'fer_manipulator'


def test_ur7e_has_ready_pose_and_no_gripper():
    config = parse_arm_config(load('ur7e'))
    assert config.gripper.type == 'none'
    assert 'ready' in config.named_poses
    assert len(config.joints) == 6


def test_unknown_joint_in_named_pose():
    data = load('ur7e')
    data['named_poses']['bad'] = {'not_a_joint': 0.0}
    with pytest.raises(ConfigError, match='unknown joints'):
        parse_arm_config(data)


def test_bad_gripper_type():
    data = load('fer')
    data['gripper']['type'] = 'magnet'
    with pytest.raises(ConfigError, match='gripper'):
        parse_arm_config(data)


def test_missing_key():
    data = load('fer')
    del data['ee_link']
    with pytest.raises(ConfigError, match='ee_link'):
        parse_arm_config(data)
