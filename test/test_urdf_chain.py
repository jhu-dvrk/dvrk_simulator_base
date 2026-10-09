"""Unit tests for UrdfChain forward kinematics and analytic Jacobian."""

import numpy as np
import pytest

from dvrk_simulator_base.urdf_chain import UrdfChain


def test_urdf_chain_simple_revolute_and_prismatic(tmp_path):
    urdf = tmp_path / "simple_arm.urdf"
    urdf.write_text("""<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="world"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tool"/>

  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="joint2" type="prismatic">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
  </joint>

  <joint name="fixed_tip" type="fixed">
    <parent link="link2"/>
    <child link="tool"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
""", encoding="utf-8")

    chain = UrdfChain(
        urdf_path=urdf,
        tool_link="tool",
        joint_names=("joint1", "joint2"),
        base_position=(0.0, 0.0, 0.0),
        base_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )

    assert chain.joint_count == 2
    pos, rot, jac = chain.forward(np.array([0.0, 0.0]))
    # Expected position: origin (0, 0, 1) + (1, 0, 0) + (0.1, 0, 0) = (1.1, 0, 1.0)
    np.testing.assert_allclose(pos, [1.1, 0.0, 1.0])
    np.testing.assert_allclose(rot, np.eye(3))
    # Jacobian shape: 6 x 2 and values at zero configuration
    assert jac.shape == (6, 2)
    expected_jac_zero = np.array([
        [0.0, 1.0],
        [1.1, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [1.0, 0.0],
    ])
    np.testing.assert_allclose(jac, expected_jac_zero)

    # Nonzero joint configuration: joint1 rotated pi/2 around z, joint2 prismatic 0.5 along x
    q_nonzero = np.array([np.pi / 2.0, 0.5])
    pos_nz, rot_nz, jac_nz = chain.forward(q_nonzero)

    # Expected position: joint1 rotated 90 deg around z, joint2 extended by 0.5 along rotated x (y-axis)
    # Total arm length: 1.0 + 0.5 (prismatic) + 0.1 (fixed tip) = 1.6 along y
    np.testing.assert_allclose(pos_nz, [0.0, 1.6, 1.0], atol=1e-7)
    expected_rot_nz = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    np.testing.assert_allclose(rot_nz, expected_rot_nz, atol=1e-7)

    # Expected Jacobian at q = [pi/2, 0.5]:
    # Col 0 (joint1 revolute z): axis=(0,0,1) x (pos - (0,0,1)) = (0,0,1) x (0, 1.6, 0) = (-1.6, 0, 0)
    #                          rotational = (0, 0, 1)
    # Col 1 (joint2 prismatic): axis rotated to (0, 1, 0)
    #                          rotational = (0, 0, 0)
    expected_jac_nz = np.array([
        [-1.6, 0.0],
        [0.0, 1.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
        [1.0, 0.0],
    ])
    np.testing.assert_allclose(jac_nz, expected_jac_nz, atol=1e-7)

    # Numerical differentiation (finite differences) to verify linear Jacobian
    eps = 1e-6
    for i in range(2):
        dq = np.zeros(2)
        dq[i] = eps
        pos_plus, _, _ = chain.forward(q_nonzero + dq)
        pos_minus, _, _ = chain.forward(q_nonzero - dq)
        numeric_j_lin = (pos_plus - pos_minus) / (2 * eps)
        np.testing.assert_allclose(jac_nz[:3, i], numeric_j_lin, atol=1e-5)


def test_urdf_chain_validation_errors(tmp_path):
    urdf = tmp_path / "bad.urdf"
    urdf.write_text("""<?xml version="1.0"?>
<robot name="bad">
  <link name="world"/>
  <link name="link1"/>
  <joint name="j1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 0 0"/>
  </joint>
</robot>
""", encoding="utf-8")

    # Missing tool link
    with pytest.raises(ValueError, match="tool link 'missing' is absent"):
        UrdfChain(urdf, "missing", ("j1",), (0, 0, 0), (0, 0, 0, 1))

    # Zero axis
    with pytest.raises(ValueError, match="zero URDF joint axis"):
        UrdfChain(urdf, "link1", ("j1",), (0, 0, 0), (0, 0, 0, 1))
