## Empowering Robots to See and Understand: A Vision-Language Model Powered by Jetson AGX and Isaac Sim

### Overview
This project enables robots to interpret visual inputs using a vision-language model. Powered by the high-performance capabilities of Jetson AGX and Isaac Sim, the setup allows for efficient image captioning and contextual understanding, empowering robots to see, understand, and respond to their surroundings in real-time.

### Prerequisites
Before setting up, make sure you have:
- **A Jetson Device**: Jetson AGX Orin (64GB), Orin Nano (8GB), etc.
- **JetPack Version**: JetPack 5 or 6
- **NVMe SSD** for storing container images and models.

### Setup Instructions
1. **Clone and Install `jetson-containers`**:
   ```bash
   git clone https://github.com/dusty-nv/jetson-containers
   cd jetson-containers
   bash install.sh
   ```

2. **Build the Container**:
   ```bash
   make
   ```

3. **Start the Container**:
   Follow the instructions in the repository to run your container. Make sure you have the appropriate environment set up for your ROS2 workspace.

4. **Run the Container with Mounted Volume**:
   Mount your `ros2_workspace` to the container and use the `nano_llm_ros2_kb` model. This will allow you to interact with your workspace and run the Vision-Language model.
   ```bash
   jetson-containers run -v ~/ros2_workspace:/ros2_workspace $(kabilankb/nano_llm_ros2_kb)
   ```

5. **Integrate with Isaac Sim**:
   After the container is up and running, you can integrate the system with Isaac Sim for real-time robotics simulation. This step will involve setting up the necessary nodes in your ROS2 workspace, such as object detection or vision-language tasks, depending on your use case.

---
