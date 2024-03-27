# Table of Contents

1. [Installation and Usage Guide for IK Solver, IK Interface, and IK Subscriber](#installation-and-usage-guide-for-ik-solver-ik-interface-and-ik-subscriber)
    1. [Prerequisites](#prerequisites)
    2. [Installation](#installation)
    3. [Usage](#usage)
        1. [IK Solver](#ik-solver)
        2. [IK Subscriber](#ik-subscriber)
    4. [Notes](#notes)
    5. [Troubleshooting](#troubleshooting)
    6. [Additional Resources](#additional-resources)
    7. [Contribution](#contribution)
    8. [License](#license)
    9. [Conclusion](#conclusion)


# Installation and Usage Guide for IK Solver, IK Interface, and IK Subscriber

This README provides a step-by-step guide on how to install and use the IK Solver, IK Interface, and IK Subscriber within a ROS 2 environment. The IK Solver calculates inverse kinematics for robots, IK Interface provides the necessary service definitions, and IK Subscriber facilitates interaction with the IK Solver through file-based pose requests.

## Prerequisites

- ROS 2 (Foxy or newer recommended)
- A working ROS 2 workspace

## Installation

1. **Clone the Repositories:**
   Navigate to the `src` directory of your ROS 2 workspace and clone the repositories for `ik_solver`, `ik_interface`, and `ik_subscriber`.

    ```bash
    cd <path-to-your-ros2-workspace>/src
    git clone <ik_solver-repository-url>
    git clone <ik_interface-repository-url>
    git clone <ik_subscriber-repository-url>
    ```

2. **Build the Packages:**
   Return to the root of your ROS 2 workspace and use `colcon` to build the packages.

    ```bash
    cd <path-to-your-ros2-workspace>
    colcon build --packages-select ik_solver ik_interface ik_subscriber
    ```

3. **Source the Setup Script:**
   Source the generated setup script to make the packages available in your environment.

    ```bash
    source install/setup.bash
    ```

## Usage

### IK Solver

- **Configure the Robot Parameters:**
  Adjust the parameters for your robot in the YAML configuration file (e.g., `robo_params.yaml`) located within the `ik_solver` package. Example configuration:

    ```yaml
    kinematics_service:
      ros__parameters:
        a1: 0.720
        a2: -0.225
        b: 0.000
        c1: 0.600
        c2: 1.075
        c3: 1.280
        c4: 0.235
    ```

- **Launch IK Solver:**
  Run the IK Solver node with the specified parameter file.

    ```bash
    ros2 run ik_solver ik_solver_node --ros-args --params-file <path-to-your-ros2-workspace>/src/ik_solver/config/robo_params.yaml
    ```

### IK Subscriber

- **Launch IK Subscriber:**
  In a new terminal (make sure your environment is sourced correctly), start the IK Subscriber node.

    ```bash
    ros2 run ik_subscriber ik_subscriber
    ```

- **Modify Pose Data:**
  To trigger a new inverse kinematics calculation, modify the pose data in the `position.txt` file within the `ik_subscriber` package. Example pose data:

    ```text
    position_x: 0.5
    position_y: 0.5
    position_z: 0.5
    orientation_x: 0.5
    orientation_y: 0.5
    orientation_z: 0.5
    orientation_w: 0.5
    ```

  Once you save changes to the `position.txt` file, the IK Subscriber automatically sends a new service request to the IK Solver based on the updated pose data.

## Notes

- Ensure both the IK Solver and IK Subscriber nodes are running before making changes to the `position.txt`.
- Monitor the ROS 2 logs for feedback and the results of the IK calculations.

By following this guide, you will be able to install and apply the IK Solver, IK Interface, and IK Subscriber in your ROS 2 project successfully.

## Troubleshooting

While working with the IK Solver, IK Interface, and IK Subscriber, you might encounter some issues. Here are some common problems and their solutions:

### IK Solver Does Not Start
- **Issue**: The IK Solver service does not start or crashes immediately after starting.
- **Solution**: Check the ROS 2 console output for any error messages. Ensure all dependencies are correctly installed and the `robo_params.yaml` file is correctly formatted and located in the specified path.

### No Response to Pose Changes
- **Issue**: Modifying `position.txt` does not trigger a new IK calculation.
- **Solution**: Ensure the IK Subscriber is running and properly monitoring the `position.txt` file. Verify the file path is correct, and the ROS 2 environment is sourced in each terminal session.

### Incorrect IK Calculations
- **Issue**: The IK calculations do not match expected results.
- **Solution**: Verify the parameters in `robo_params.yaml` accurately reflect your robot's physical characteristics and the pose data in `position.txt` is within the robot's operational range.

## Additional Resources

For more detailed information on ROS 2 and inverse kinematics solutions, consider the following resources:

- [ROS 2 Documentation](https://docs.ros.org/en/foxy/index.html) - Official documentation for ROS 2.
- [Robot Kinematics](https://en.wikipedia.org/wiki/Robot_kinematics) - Wikipedia article providing a general overview of robot kinematics.
- [Understanding ROS2 Services](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/) - A tutorial on using ROS 2 services, which can be helpful for customizing the IK Solver.

## Contribution

Contributions to the IK Solver, IK Interface, and IK Subscriber packages are welcome. Whether it's adding new features, improving documentation, or reporting issues, your input is valuable. Check the respective repository guidelines for contributing.

## License

The IK Solver, IK Interface, and IK Subscriber are open-source projects. The license can vary between these projects, so please refer to each project's repository for specific licensing information.

## Conclusion

This guide provided steps to install, configure, and use the IK Solver, IK Interface, and IK Subscriber within a ROS 2 environment. By following this guide, you should be able to integrate inverse kinematics calculations into your robotic projects effectively. If you encounter any issues or have suggestions for improvement, please contribute to the project repositories.

Thank you for using the IK Solver, IK Interface, and IK Subscriber. Happy coding!
