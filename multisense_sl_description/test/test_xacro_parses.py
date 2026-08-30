# Copyright 2026 Open Source Robotics Foundation
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

"""Run xacro on each top-level robot description and sanity-check the URDF it produces."""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PACKAGE_DIR = Path(__file__).resolve().parent.parent
ROBOTS_DIR = PACKAGE_DIR / 'robots'

XACRO_FILES = sorted(ROBOTS_DIR.glob('*.urdf.xacro'))


def run_xacro(xacro_file: Path) -> str:
    result = subprocess.run(
        ['xacro', str(xacro_file)],
        capture_output=True, text=True, cwd=str(PACKAGE_DIR),
    )
    assert result.returncode == 0, (
        f'xacro failed on {xacro_file.name}:\n{result.stderr}'
    )
    return result.stdout


def assert_well_formed_urdf(urdf_xml: str, xacro_file: Path) -> None:
    root = ET.fromstring(urdf_xml)
    assert root.tag == 'robot', f'{xacro_file.name}: root element is <{root.tag}>, not <robot>'

    link_names = {link.get('name') for link in root.findall('link')}
    assert link_names, f'{xacro_file.name}: no <link> elements found'

    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        parent = joint.find('parent')
        child = joint.find('child')
        assert parent is not None and child is not None, (
            f'{xacro_file.name}: joint {joint_name} missing parent/child'
        )
        assert parent.get('link') in link_names, (
            f"{xacro_file.name}: joint {joint_name} parent link "
            f"'{parent.get('link')}' not defined"
        )
        assert child.get('link') in link_names, (
            f"{xacro_file.name}: joint {joint_name} child link "
            f"'{child.get('link')}' not defined"
        )


@pytest.mark.parametrize('xacro_file', XACRO_FILES, ids=lambda p: p.name)
def test_xacro_produces_valid_urdf(xacro_file):
    urdf_xml = run_xacro(xacro_file)
    assert_well_formed_urdf(urdf_xml, xacro_file)


def test_found_expected_xacro_files():
    names = {f.name for f in XACRO_FILES}
    assert names == {
        'multisense_sl_description.urdf.xacro',
        'multisense_sl_cpu_description.urdf.xacro',
        'multisense_sl_on_box.urdf.xacro',
    }
