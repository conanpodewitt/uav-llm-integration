# uav-llm-integration

**Conan Dewitt (22877792) - [GitHub](https://github.com/conanpodewitt)**

A ROS 2‑based framework integrating OpenAI’s LLM for high‑level mission planning with both simulated (Gazebo) and real Pioneer 3‑AT hardware. The system consists of five packages, with three of them being the focus of this project:

- **llm_integration**
    - Interacts with the OpenAI API to generate and execute motion plans
    - Includes:
        - [`llm_node`](src/llm_integration/llm_integration/llm_node.py) (core planning & execution)
        - [`ui_node`](src/llm_integration/llm_integration/ui_node.py) (text CLI/GUI input)
        - [`camera_caption_node`](src/llm_integration/llm_integration/camera_caption_node.py) (image -> caption for context)

- **deadman_safety**
    - Ensures collision avoidance and a “deadman” joystick override
    - Includes:
        - [`custom_joy_node`](src/deadman_safety/deadman_safety/custom_joy_node.py) (reads controller via evdev -> publishes JSON)
        - [`deadman_node`](src/deadman_safety/deadman_safety/deadman_node.py) (merges LLM commands, joystick, LIDAR safety)

- **spoof_tools**
    - Provides tools for testing system resilience against sensor spoofing and plan poisoning
    - Includes:
        - [`poison_node`](src/spoof_tools/spoof_tools/poison_node.py) (intercepts `/text_in` commands for testing plan injection)
        - [`mirage_node`](src/spoof_tools/spoof_tools/mirage_node.py) (intercepts `/camera` feed for camera spoofing tests)

## Prerequisites

- Linux machine (required by [`init.sh`](init.sh)/[`run.sh`](run.sh)) - Other platforms not supported
- Docker
- USB serial (`/dev/ttyUSB0`), USB video (`/dev/video0`), and ethernet LiDAR (SICK TIM-7) when using "actual"
- Joystick (`/dev/input/js0`) for both simulation and "actual"

## Environment Configuration

1. Copy or create a file named `.env` in the project root containing your OpenAI key: `LLM_API_KEY=sk-<your_openai_api_key>`
2. In [`conf/.env.conf`](conf/.env.conf) define system parameters
3. [`conf/system_prompt.txt`](conf/system_prompt.txt) holds your system‑level prompt for the LLM

## Initial Hardware Setup

Before launching, grant permissions and configure interfaces. Run **with** `sudo`:

```bash
sudo ./init.sh
```

You can also go through this file and run these commnads manually if you wish.

## Build & Launch

From project root, simply run:

```bash
./run.sh
```

What it does:
1. Verifies Linux host, `.env` & [`conf/.env.conf`](conf/.env.conf) exist
2. Builds a Docker image `uav-llm-integration` with your parameters
3. Detects `/dev/ttyUSB0` & `/dev/video0` - if present, runs hardware‑enabled container; otherwise sim‑only
4. Mounts X11 for RViz and GUI, exposes devices as needed

## Choosing Simulation vs Hardware

Once inside the container (or on your host if you source ROS 2):

- **Simulation**
    ```bash
    ros2 launch master_launch sim.launch.py
    ```
    - Brings up Gazebo, ros_gz_bridge, ..., rest of the system stack

- **Actual Hardware**
    ```bash
    ros2 launch master_launch actual.launch.py
    ```  
    - Starts ARIA driver, LiDAR, ..., rest of the system stack

## Workflow Overview

1. **User Input** ([`ui_node`](src/llm_integration/llm_integration/ui_node.py)) -> `/text_in` topic
2. **LLM Plan Generation** ([`llm_node`](src/llm_integration/llm_integration/llm_node.py)) -> `/llm_cmd` & `/plan` topics
3. **Safety & Joystick Merge** ([`deadman_node`](src/deadman_safety/deadman_safety/deadman_node.py)) -> verification, then `/cmd_vel` topic
4. **Execution**
    - Sim: Gazebo <-> `/cmd_vel` via ros_gz_bridge
    - Real: `ariaNode` -> serial commands to Pioneer 3‑AT

## Project Structure

```text
.
├── conf/
│   ├── .env.conf            # System parameters
│   └── system_prompt.txt    # LLM system prompt
├── .env                     # LLM_API_KEY
├── Dockerfile               # Builds system container
├── init.sh                  # Hardware permission script (run with sudo)
├── run.sh                   # Build & launch script
└── src/
    ├── llm_integration/     # LLM + UI + caption nodes
    ├── deadman_safety/      # Safety & custom joy nodes
    ├── uav_sim/             # Gazebo world + bridge
    ├── uav_actual/          # Hardware drivers
    ├── spoof_tools/         # Plan poisoning and camera spoofing tools
    └── master_launch/       # Combined sim and actual launch files
```