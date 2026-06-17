# Third-Party License Notices

The `open-robin` project code is licensed under AGPL-3.0-only. See the repository root [`LICENSE`](LICENSE) file.

This file documents third-party or template-origin license notices that may appear inside the repository and are not project-level license declarations.

## ROS 2 / ament Template Test Files

Several ROS 2 package test files are generated from standard ROS 2 / ament package templates and carry Open Source Robotics Foundation copyright headers under Apache-2.0.

These files are retained as template/linting support files and keep their original notices:

- `vulcanexus_ws/src/robin_core_sensor/test/test_copyright.py`
- `vulcanexus_ws/src/robin_core_sensor/test/test_flake8.py`
- `vulcanexus_ws/src/robin_core_sensor/test/test_pep257.py`
- `vulcanexus_ws/src/robin_hardware_fronius/test/test_copyright.py`
- `vulcanexus_ws/src/robin_hardware_fronius/test/test_flake8.py`
- `vulcanexus_ws/src/robin_hardware_fronius/test/test_pep257.py`
- `vulcanexus_ws/src/robin_hardware_garmo/test/test_copyright.py`
- `vulcanexus_ws/src/robin_hardware_garmo/test/test_flake8.py`
- `vulcanexus_ws/src/robin_hardware_garmo/test/test_pep257.py`
- `vulcanexus_ws/src/robin_hardware_ur/test/test_copyright.py`
- `vulcanexus_ws/src/robin_hardware_ur/test/test_flake8.py`
- `vulcanexus_ws/src/robin_hardware_ur/test/test_pep257.py`
- `vulcanexus_ws/src/robin_simulation/test/test_copyright.py`
- `vulcanexus_ws/src/robin_simulation/test/test_flake8.py`
- `vulcanexus_ws/src/robin_simulation/test/test_pep257.py`
- `vulcanexus_ws/src/robin_core_bringup/test/test_copyright.py`
- `vulcanexus_ws/src/robin_core_bringup/test/test_flake8.py`
- `vulcanexus_ws/src/robin_core_bringup/test/test_pep257.py`

The Apache-2.0 notices in those files apply to the template files themselves. They do not change the project-level AGPL-3.0-only license declaration for `open-robin` code.

## JavaScript Dependencies

JavaScript dependency license declarations are recorded in `package-lock.json` files, for example MIT, Apache-2.0, BSD, ISC, MPL, and other dependency licenses.

Those entries describe third-party packages installed by npm. They are intentionally not rewritten when aligning the repository's own project license metadata.

## Python Dependencies

Python dependency license metadata is provided by the respective upstream packages and their package indexes. The project-level license metadata in `pyproject.toml` applies to `open-robin`, not to third-party dependencies.
