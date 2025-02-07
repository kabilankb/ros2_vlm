---

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

5. **Launch the Vision-Language Model in ROS2**:
   Now that the container is running, you can launch your Vision-Language Model (VLM) ROS2 node with the following command:
   ```bash
   ros2 launch ros2_vlm vlm_launch.py
   ```
   This will start the ROS2 launch file for your vision-language model, enabling interaction between the robot and its visual environment.

6. **Integrate with Isaac Sim**:
   After launching the ROS2 node, you can integrate the system with Isaac Sim for real-time robotics simulation. This step will involve setting up the necessary nodes in your ROS2 workspace, such as object detection or vision-language tasks, depending on your use case.


You can add a call-to-action at the end of your blog post to encourage readers to seek more information. Here's how you can structure it:

---

**For more information**, you can check out the full setup and details in the GitHub repository or reach out to me through my [Medium blog](https://medium.com/@kabilankb2003/empowering-robots-to-see-and-understand-a-vision-language-model-powered-by-jetson-agx-and-isaac-f797c076b94f) for any questions or clarifications.

Feel free to follow me for more updates and tutorials on robotics, AI, and computer vision!

---
