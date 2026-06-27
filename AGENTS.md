# QUESTiX Agent Guide

## Project overview

- QUESTiX is a repository for robot control and ISO image builds targeting ROS 2 Jazzy on Ubuntu 24.04.
- The repository contains a mix of C++ ROS 2 packages, launch/config files, Ansible assets, systemd units, and the FastAPI-based `robot_manager`.

## Project naming and documentation style

- Use `QUESTiX` as the project name in prose, headings, summaries, PR bodies, and generated documentation.
- Do not mechanically rename repository names, package names, file paths, URLs, commands, service names, launch arguments, environment variables, or code identifiers.
- Preserve existing lowercase identifiers such as `questix_launcher`, `questix_robot`, and repository/path names unless the user explicitly requests a code-level rename.
- When generating documentation, distinguish between the product/project name `QUESTiX` and implementation identifiers such as package names or file paths.

## Environment baseline

- AMD64 development environment: Ubuntu 24.04 + ROS 2 Jazzy.
- ARM64 Raspberry Pi 5 robotics kit: Ubuntu 24.04 + ROS 2 Jazzy.
- WSL and desktop environments are useful for reading, editing, static checks, and non-hardware tests.
- Raspberry Pi 5 hardware validation is authoritative for GPIO, UART, I2C, SPI, systemd, robot startup, and controller integration.

## Main directories

- `motor_control_lib/`: Shared library for motor control.
- `motor_control_app/`: ROS 2 nodes and components for drive, shot, single DDT, and related motor-control applications.
- `joy_controller/`, `uart_joy_driver/`, `joy_gate/`, `gpio_reader/`: Input, gating, and GPIO-related packages.
- `operation_manager/`: Operational state management.
- `launcher/`: Integrated entry point for ROS launch files. The ROS package name is `questix_launcher`.
- `description_launch/`: URDF, RViz, and xacro assets.
- `ansible/`, `scripts/`, `systemd/`: OS setup, ISO build tooling, and resident services.
- `scripts/robot_manager/`: FastAPI web management UI.

## Pre-work checks

- Always verify consistency across `package.xml`, `CMakeLists.txt`, `launch/`, and `config/` before and after changes.
- Do not assume directory names always match ROS package names. For example, `launcher/` maps to the ROS package name `questix_launcher`.
- Hardware-dependent code is likely impossible to validate fully without the physical robot or target hardware.

## Pull request titles

- Pull request titles must follow Conventional Commits because this repository runs a semantic pull request title check.
- Use the format: `<type>: <lowercase summary>`.
- Common types include `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, and `revert`.
- Prefer concise summaries after the colon.
- Examples:
  - `fix: resolve recursive loop in setup_dev role vars`
  - `ci: align ISO workflow with Ubuntu 24.04 and ROS 2 Jazzy`
  - `docs: add AI agent guidance`
  - `chore: sync upstream main through PR #56`
- Before opening or updating a PR, ensure the title passes the semantic pull request check.

## C++ / ROS 2 implementation policy

- Use C++17 unless an existing package explicitly specifies a different standard.
- Follow `.clang-format`.
- Prefer `ament_cmake_auto` for ROS 2 packages.
- For component implementations, keep `rclcpp_components_register_nodes` and `RCLCPP_COMPONENTS_REGISTER_NODE` entries consistent, including registration names and class names.
- Update YAML parameters, launch arguments, and in-node `declare_parameter` / `get_parameter` usage together.

## C++ formatting

- C++ formatting is checked by CI with `ament_clang_format` and the repository `.clang-format`.
- Before pushing C++ changes, run the CI-equivalent format check:

  ```bash
  source /opt/ros/jazzy/setup.bash
  git ls-files -z -- '*.cpp' '*.hpp' | xargs -0 -r ament_clang_format --config .clang-format
  ```

- To reformat only the C++ files touched by a PR, run:

  ```bash
  source /opt/ros/jazzy/setup.bash
  ament_clang_format --config .clang-format --reformat <changed-file-1> <changed-file-2>
  ```

- Do not reformat unrelated C++ files just to fix a PR unless the user explicitly asks for a repository-wide formatting change.
- If `ament_clang_format` is missing on Ubuntu 24.04 / ROS 2 Jazzy, install it with:

  ```bash
  sudo apt update
  sudo apt install -y ros-jazzy-ament-clang-format
  ```

## Command safety

- Do not run ISO builds, QEMU tests, package installation, permission changes, systemd commands, or hardware-facing commands unless explicitly requested.
- Do not run destructive commands such as `rm -rf`, broad `chmod`, or broad `chown`.
- Explain the expected impact before changing Ansible, shell scripts, systemd units, UART/GPIO behavior, or ISO build behavior.
- Treat `systemd/questix_robot*`, `scripts/build-iso.sh`, and `ansible/playbooks/*.yaml` with extra care because they can have significant effects on real hardware or the OS.

## Ansible and setup validation

- Treat Ansible, setup scripts, ISO provisioning, and robot startup as OS-impacting areas.
- Separate static validation from state-changing validation.
- Safe default validation candidates:
  - `git diff --check`
  - `yamllint ansible/`
  - `ansible-playbook ansible/playbooks/setup_kit.yaml --syntax-check -i localhost,`
  - `ansible-playbook ansible/playbooks/setup_dev.yaml --syntax-check -i localhost,`
  - `bash -n setup.sh setup_dev.sh scripts/install-robot-manager.sh scripts/apply-ansible-config.sh`
  - sandbox-safe `make test-ansible`
- Do not run `setup.sh`, `setup_dev.sh`, package installation, systemd commands, GPIO/UART commands, ISO build, or QEMU unless explicitly requested.
- If a real setup run is explicitly requested, record:
  - branch and commit
  - host OS and architecture
  - execution user
  - whether `/root/ros2_ws` or `/root/.bashrc` was accidentally touched
  - ROS 2, colcon, rosdep, and vcs post-checks
  - skipped hardware/system checks

## Ansible check mode limits

- Do not assume `ansible-playbook --check --diff` fully validates installer playbooks.
- Check mode can skip modules that do not support it, and later tasks may fail if they depend on registered variables from skipped tasks.
- `check_mode: false` is acceptable only for read-only discovery tasks that must run to provide registered variables, such as:
  - `whoami`
  - HTTP GET metadata lookups
- Do not add `check_mode: false` to package installation, `get_url` downloads, file writes, systemd operations, GPIO/UART access, or other state-changing tasks just to make check mode pass.
- `get_url` may validate the URL in check mode without downloading the file body; do not treat a later missing downloaded file as proof that the real run will fail.
- When check mode reaches this kind of installer limitation, report it clearly and ask before proceeding to a real setup run.

## AMD64 setup_dev execution boundary

- `ansible/playbooks/setup_dev.yaml` is intended for an AMD64 development machine where a regular user runs the playbook with sudo/become.
- Keep safeguards that prevent accidental `/root/ros2_ws` or `/root/.bashrc` configuration.
- Do not weaken `/home/...` assertions to support root-driven local or chroot execution unless explicitly requested.
- Treat AMD64 root/chroot provisioning as a separate ISO workflow design issue.
- Keep repository-root `ansible.cfg` and copied-subtree `ansible/ansible.cfg` roles distinct:
  - root `ansible.cfg`: repository-root local setup and CI commands
  - `ansible/ansible.cfg`: copied into `/tmp/ansible` for chroot provisioning and should resolve sibling `roles/`

## Validation commands

The following commands are standard validation candidates. Whether they can run depends on the local environment, including ROS 2 availability, Ansible/lint tooling, hardware access, and OS permissions. Hardware-dependent behavior still requires real robot/Raspberry Pi validation.

- `git diff --check`
- `colcon build --symlink-install` (run from the workspace root)
- `colcon test` (run from the workspace root)
- For Ansible changes: `make test-ansible`
- For Ansible/YAML changes: `yamllint ansible/`
- For GitHub Actions changes: `yamllint .github/workflows/` and `actionlint` if available.

## Cross-reference caution

- When changing package names, launch-file references to `find-pkg-share`, or dependency package names, perform a cross-package search within this repository and update all related references.
