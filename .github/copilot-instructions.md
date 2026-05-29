# Copilot Instructions for Questix

## Repository baseline

- This repository is based on ROS 2 Jazzy, Ubuntu 24.04, C++17, and `ament_cmake_auto`.
- Use `ament_cmake_auto` as the default build style for ROS 2 packages in this repository.

## C++ completion conventions

- Follow the repository `.clang-format` when completing, generating, or modifying C++ code.
- Match the existing code style for `rclcpp` logging, including logger selection, severity, message wording, and formatting.
- Keep node names, topic names, and parameter names consistent with the existing YAML and launch files.

## ROS 2 package additions and changes

- Update `package.xml` dependencies whenever adding or changing a ROS 2 package.
- In `CMakeLists.txt`, verify the relationship between `ament_auto_find_build_dependencies()`, `ament_auto_add_library()`, `ament_auto_add_executable()`, and `rclcpp_components_register_nodes()`.
- Confirm install targets such as `ament_auto_package(INSTALL_TO_SHARE launch config)` so launch files, config files, and other share resources are installed correctly.

## Launch and config completion notes

- `launcher/launch/questix_core.launch.xml` is the integrated launch entry point for Questix.
- Some launch and config code uses `find-pkg-share questix_launcher`; verify references when changing package names or paths.
- The environment variables `ENABLE_LIDAR`, `ENABLE_SHOT`, `ENABLE_DRIVE`, `ENABLE_GPIO_REF`, `ENABLE_RVIZ`, and `CONTROLLER_TYPE` are related to the launch flow and the systemd / robot_manager integration.

## Ansible, shell, and systemd changes

- Follow the rules in `.ansible-lint.yml` and `.yamllint.yml` when changing Ansible or YAML files.
- When changing Ansible, shell, or systemd files, check the relationship between `ansible/playbooks/setup_dev.yaml`, `ansible/playbooks/setup_kit.yaml`, `systemd/questix_robot.service`, and `scripts/install-robot-manager.sh`.
