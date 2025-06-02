# NAV2-USV-Packages

This repository can be used to create a vscode-docker ROS2 Humble dev environment on any host OS.  
Environment setup is based on [ROS2 Community Guide](https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html?highlight=vscode)

## Installation

1. Install Docker
2. Install Remote Development extension in vscode
3. Clone and open this repository in vscode
4. Open Command Palette and run "Dev Containers: Rebuild and Reopen in Container"
5. Wait for container to install


## Instruction to Build at first time

Run the build script:
    ```bash
    ./scripts/build-all.sh
    ```


# Instruction to Run

Run the following command:
- To launch the stack:
    ```bash
       ./usv_nav2_launch.sh
    ```

