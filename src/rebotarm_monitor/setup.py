"""Minimal setup.py kept because ament_python needs ``data_files``.

Everything else (pytest, ruff, build-system) lives in pyproject.toml.

Notes:
- ``data_files`` cannot move to pyproject.toml: ament_python relies on it to
  copy launch and config files into ``install/share/<pkg>/``.
- We register a small ``cmdclass`` for ``test`` so that ``colcon test``
  (which calls ``python setup.py test``) delegates to pytest. Without this
  hook colcon falls back to setuptools' unittest runner, which finds nothing.
"""

import sys
from glob import glob

from setuptools import find_packages, setup
from setuptools.command.test import test as TestCommand


class PytestCommand(TestCommand):
    user_options = [("pytest-args=", "a", "Arguments forwarded to pytest")]

    def initialize_options(self):
        super().initialize_options()
        self.pytest_args = ""

    def finalize_options(self):
        super().finalize_options()

    def run_tests(self):
        import shlex

        import pytest

        sys.exit(pytest.main(shlex.split(self.pytest_args) + ["test"]))


package_name = "rebotarm_monitor"

setup(
    name=package_name,
    version="0.2.2",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reBotArm Maintainers",
    maintainer_email="support@example.com",
    description="Passive external monitor for reBotArm hardware state topics.",
    license="Apache-2.0",
    cmdclass={"test": PytestCommand},
    entry_points={
        "console_scripts": [
            "monitor = rebotarm_monitor.node:main",
            "joint_state_monitor = rebotarm_monitor.node:main",
        ],
    },
)
