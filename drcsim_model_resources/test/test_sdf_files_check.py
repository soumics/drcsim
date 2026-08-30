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

import os
from pathlib import Path
import subprocess

import pytest

PACKAGE_DIR = Path(__file__).resolve().parent.parent

WORLD_FILES = sorted((PACKAGE_DIR / 'worlds').glob('*.world'))
MODEL_SDF_FILES = sorted((PACKAGE_DIR / 'gazebo_models').glob('*/model.sdf'))

# `gz sdf --check` run standalone (no gz-sim server around it) never gets
# GZ_SIM_RESOURCE_PATH wired up automatically -- that's set up by gz-sim's
# own runtime, not by the generic sdformat CLI -- so every model:// URI
# fails to resolve ("Tried to use callback in sdf::findFile(), but the
# callback is empty") unless we point it at the models directory
# ourselves, matching this package's own GZ_SIM_RESOURCE_PATH environment
# hook (hooks/drcsim_model_resources.dsv.in).
GZ_SDF_CHECK_ENV = dict(os.environ)
GZ_SDF_CHECK_ENV['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
    filter(None, [
        str(PACKAGE_DIR / 'gazebo_models'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))


def run_gz_sdf_check(sdf_file: Path) -> None:
    result = subprocess.run(
        ['gz', 'sdf', '--check', str(sdf_file)],
        capture_output=True, text=True, env=GZ_SDF_CHECK_ENV,
    )
    assert result.returncode == 0, (
        f'gz sdf --check failed on {sdf_file.relative_to(PACKAGE_DIR)}:\n'
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
