# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import subprocess
import sys
from pathlib import Path

import pytest


@pytest.mark.linter
def test_ruff():
    package_root = Path(__file__).resolve().parents[1]
    lint_result = subprocess.run(
        [sys.executable, "-m", "ruff", "check", str(package_root)],
        capture_output=True,
        text=True,
        check=False,
    )
    format_result = subprocess.run(
        [sys.executable, "-m", "ruff", "format", "--check", str(package_root)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert lint_result.returncode == 0, lint_result.stdout + lint_result.stderr
    assert format_result.returncode == 0, format_result.stdout + format_result.stderr
