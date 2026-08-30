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

"""Lint-style check: every world and model.sdf file must pass `gz sdf --check`."""

from pathlib import Path
import re
import subprocess

import pytest

PACKAGE_DIR = Path(__file__).resolve().parent.parent

WORLD_FILES = sorted((PACKAGE_DIR / 'worlds').glob('*.world'))
MODEL_SDF_FILES = sorted((PACKAGE_DIR / 'gazebo_models').glob('*/model.sdf'))

# sdf::findFile()'s URI-resolution callback is only ever registered by
# gz-sim's own runtime when it starts a server -- the standalone `gz sdf`
# CLI never sets one up, with or without GZ_SIM_RESOURCE_PATH set (that
# was tried and confirmed not to help: identical failures either way).
# So every <include><uri>model://...</uri></include> a file has reliably
# fails with "Error Code 14: ... Unable to find uri[...]" here, regardless
# of whether the referenced model actually exists -- that's a limitation
# of checking a file in isolation, not a defect in the file.
UNRESOLVED_URI_ERROR_CODE = 14

# A model that's purely a nested-model wrapper (an <include> of an
# external model plus a <plugin>, e.g. drc_vehicle/drc_vehicle_xp900
# composing in polaris_ranger_ev/xp900 and attaching the vehicle control
# plugin -- a legitimate, common SDF pattern) can't have its own links
# counted either, once the include it depends on fails to resolve for the
# same reason as above: the checker can't see inside an unresolved
# include to find the links it would otherwise contribute. That surfaces
# as secondary errors 17 (no link) and 25 (frame graph), *alongside* 14 --
# so those two are only tolerated together with 14, never on their own
# (a file with 17/25 and no 14 has a genuine standalone missing-link bug,
# like block_angle_steps/block_level_steps did before they were fixed to
# have <static>true</static>, matching their sibling block_angle_base).
UNRESOLVED_INCLUDE_CASCADE_CODES = {17, 25}

ERROR_CODE_RE = re.compile(r'Error Code (\d+):')


def run_gz_sdf_check(sdf_file: Path) -> None:
    result = subprocess.run(
        ['gz', 'sdf', '--check', str(sdf_file)],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        return

    error_codes = {int(code) for code in ERROR_CODE_RE.findall(result.stderr)}
    tolerated = {UNRESOLVED_URI_ERROR_CODE}
    if UNRESOLVED_URI_ERROR_CODE in error_codes:
        tolerated |= UNRESOLVED_INCLUDE_CASCADE_CODES
    unexpected_codes = error_codes - tolerated
    assert not unexpected_codes and error_codes, (
        f'gz sdf --check failed on {sdf_file.relative_to(PACKAGE_DIR)} '
        f'with error code(s) {sorted(error_codes) or "none (unparsed)"}, '
        f'not just the tolerated unresolved-include code(s) '
        f'{sorted(tolerated)}:\n'
        f'stdout: {result.stdout}\nstderr: {result.stderr}'
    )


@pytest.mark.parametrize(
    'sdf_file', WORLD_FILES, ids=lambda p: p.relative_to(PACKAGE_DIR).as_posix())
def test_world_file_valid(sdf_file):
    run_gz_sdf_check(sdf_file)


@pytest.mark.parametrize(
    'sdf_file', MODEL_SDF_FILES, ids=lambda p: p.relative_to(PACKAGE_DIR).as_posix())
def test_model_sdf_valid(sdf_file):
    run_gz_sdf_check(sdf_file)


def test_found_expected_file_counts():
    # Sanity check that the globs above actually found the files -- if this
    # starts failing after a repo change, the globs (not the SDF content)
    # are the first thing to check.
    assert len(WORLD_FILES) >= 40
    assert len(MODEL_SDF_FILES) >= 20
