# An End-to-End Pothole Detection and Avoidance System for Autonomous Ground Vehicles

The implementation of the system proposed in "An End-to-End Pothole Detection and Avoidance System for Autonomous Ground Vehicles" in ROS 2 Foxy/Humble, tested on the AUB NV-X1 research platform. This work was performed under the [AUB Vision & Robotics Lab](https://sites.aub.edu.lb/vrlab/).

# Installation

## Hardware Requirements

- A ZED camera (tested on ZED 1, most probably works on ZED 2), system can be tested on recorded SVOs
- A CUDA-enabled NVidia GPU (tested on GTX 1650)

## Software Requirements and Procedure

1. Make sure you are running Ubuntu 20.04/22.04 with NVidia graphics drivers enabled (<https://ubuntu.com/download/desktop>).
2. Install ROS 2 Foxy/Humble: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>
3. Configure CUDA (<https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html>) and install PyTorch with CUDA locally using pip: <https://pytorch.org/get-started/locally/>
4. Install the ZED SDK with Python: <https://www.stereolabs.com/developers/release/>
5. `pip3 install ultralytics pyserial`
6. Clone this repository, making sure to recurse the zed-interfaces submodule: `git clone --recurse-submodules git@github.com:AUBVRL/pothole-detection-avoidance-agv.git`
7. Download the model weights (linked in the Dataset and Weights section), then move the weights into `agv_ws/src/system_resources/weights` (make the directory first).
8. `cd` into the `agv_ws` folder, then build and source the workspace:

```bash
colcon build --symlink-install
. install/setup.bash
```

# Detection Model Dataset and Weights

## Dataset

The dataset used to train the model may be downloaded from [Roboflow](https://universe.roboflow.com/pothole-vsmtu/potholes-and-roads-instance-segmentation). This dataset was used in conjunction with the [HighD dataset](https://www.highd-dataset.com/), whose labels were edited to fit the YOLOv8 training label format.

## Models

The trained models are based on the YOLOv8n-seg and YOLOv8s-seg instance segmentation model.
| Model | Download |
| --- | --- |
| YOLOv8n-pothole (recommended) | [drive](https://drive.google.com/uc?id=1JF5LX9Jw1kT490nuOcYS9uMPxiKE0Jl8&export=download) |
| YOLOv8s-pothole | [drive](https://drive.google.com/uc?id=1mfwCtO5QUO8QcK-GbM2-4vC8yzChlRJO&export=download) |

# Running

Launch the system launch file:

```bash
ros2 launch system_resources full_system.launch.py
```

If connecting to the AUB AGV, run the arduino interface node as well: `ros2 run arduino_interface arduino_interface`

## Launch Arguments

| Argument | Description |
| --- | --- |
| `svo` | Runs the system on a prerecorded svo file. Give an absolute path to an svo file: `svo:=[path]`. |
| `weights` | Change the weights used for the test. Make sure to put the weights in `agv_ws/src/system_resources/weights`, and provide only the filename. |

# Citation

Citation pending.

# Acknowledgment

This work was funded by the University Research Board (URB) at AUB and the Lebanese National Council for Scientific Research (LNCSR). The authors would also like to thank Mrs. Nada Sehnaoui for donating the NV-X1 vehicle to AUB.

YOLOv8 is property of [Ultralytics](https://ultralytics.com/yolov8). The ZED Camera and ZED SDK have been developed by [Stereolabs](https://www.stereolabs.com/).

# License



# Contact

If you have any inquiries, please don't hesitate to contact any member of the team:

Nader Zantout: [GitHub](https://github.com/SpicePlusPlus) | [LinkedIn](https://www.linkedin.com/in/nader-zantout/) | `nwz05@mail.aub.edu` | `nwzantout@gmail.com`

Leen Daher: [GitHub](https://github.com/Line-D) | [LinkedIn](https://www.linkedin.com/in/leen-daher-513103216/) | `lkd03@mail.aub.edu`

Yara Ghaddar: [GitHub](https://github.com/yaraghaddar) | [LinkedIn](https://www.linkedin.com/in/yara-ghaddar-27b41518b/) | `yag04@mail.aub.edu`

Omar Chaaban: [GitHub](https://github.com/OmSh01) | `oms20@mail.aub.edu`