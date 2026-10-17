# ROS 无人车视觉巡航与自主任务执行

本项目基于 ROS 构建无人车巡航任务流程：机器人依次前往四个读题点，采集摄像头画面并识别题目，根据识别出的黄色格编号访问对应任务点，最后返回终点。仓库同时包含底盘、建图定位、导航、视觉、语音及 Gazebo 场景等模块。



## 任务流程

1. 启动底盘、雷达、摄像头、地图、AMCL 和 `move_base`。
2. 等待视觉节点就绪，并通过开始口令或 `/start_mission` 消息启动任务。
3. 依次导航至四个读题点；每到一处发送视觉请求，读取黄色格编号。
4. 按识别顺序访问四个对应任务点，最后前往终点。
5. 通过语音播报和 `/patrol/stage` 发布任务状态；导航或识别失败时进入 `ABORTED` 状态。

当前代码接受的目标编号为 `31、32、33、40、41、42、49、50、51`。四个答案允许重复，重复目标仍会按顺序访问。

## 主要目录

| 目录 | 作用 |
| --- | --- |
| `src/robot_slam/` | 巡航主控、任务点配置、导航启动文件和地图 |
| `src/competition_ai/` | ROS 桥接、视觉分析、红色目标检测、语音服务后端 |
| `src/abot_bringup/`、`src/abot_driver/` | 底盘和传感器启动、设备驱动 |
| `src/abot_model/` | 机器人模型和 Gazebo 场景 |
| `src/TTS_audio/` | 语音服务接口 |
| 其余 `src/` 包 | 配套或实验功能，是否参与本任务应以实际启动文件为准 |

本任务的关键入口是 `src/robot_slam/launch/patrol_all.launch`，主控逻辑位于 `src/robot_slam/scripts/patrol_controller.py`，任务坐标与超时参数位于 `src/robot_slam/config/patrol.yaml`。

## 系统接口

| 接口 | 用途 |
| --- | --- |
| `/move_base` | 导航目标 Action |
| `/usb_cam/image_raw` | 摄像头图像，默认输入话题 |
| `/patrol/vision/request` | 主控发起视觉识别请求 |
| `/patrol/vision/result` | 视觉结果，带运行和请求标识 |
| `/start_mission` | 启动任务 |
| `/patrol/stage` | 当前任务阶段 |
| `/tts_service` | 语音播报服务 |

## 环境与依赖

- ROS 1 catkin 工作区；源码提供的桥接层兼容 ROS 使用的 Python 2/3，AI 后端单独使用 Python 3。请按实际 ROS 发行版准备环境。
- 运行所需 ROS 包包括 `move_base`、`map_server`、`amcl`、`usb_cam`，以及本工作区内的驱动、模型和消息服务包。
- AI 后端的 Python 依赖见 `src/competition_ai/requirements.txt`；语音唤醒还依赖 `src/robot_slam/resources/` 下的离线资源及麦克风设备。
- 视觉分析需要 `ARK_API_KEY`；语音合成需要 `VOLC_TTS_APPID` 和 `VOLC_TTS_TOKEN`。这些凭据应通过环境变量提供，不能提交到仓库。
- `patrol_all.launch` 面向实体车，默认使用 `/dev/abotbase`、`/dev/rplidar` 和 `/dev/video0`，并设置 `use_sim_time=false`。Gazebo 文件存在于源码中，但不能据此认为这套实体车启动文件可直接用于仿真。

## 构建与启动

以下命令以仓库根目录为 catkin 工作区根目录为例，在已安装 ROS 的 Linux 环境中执行：

```bash
cd /path/to/race_ROS_Test
catkin_make
source devel/setup.bash
```

运行前，检查 `src/robot_slam/config/patrol.yaml` 的读题点、目标点和终点坐标是否与当前地图一致，并设置 AI 服务凭据及适用的 Python 解释器：

```bash
export ARK_API_KEY='你的密钥'
export VOLC_TTS_APPID='你的应用 ID'
export VOLC_TTS_TOKEN='你的令牌'
export AI_PYTHON='/path/to/python3'
export ROS_PYTHON='python'
roslaunch robot_slam patrol_all.launch
```

`AI_PYTHON` 应指向安装了 `src/competition_ai/requirements.txt` 中依赖的 Python 3 环境；`ROS_PYTHON` 应指向当前 ROS 环境可用的 Python。设备路径、地图和摄像头格式可通过 `patrol_all.launch` 的参数调整。不要把上述真实凭据写进脚本或 README。

仅在其他必需节点已分别启动时，可使用 `roslaunch robot_slam patrol_controller.launch` 单独运行主控节点。

## 当前实现中的可靠性处理

- 导航等待设置超时并检查 `move_base` 的终态；失败时取消目标。
- 每次视觉请求携带 `run_id`、`request_id` 和 `task_id`，减少旧结果误入当前任务的风险。
- 视觉结果仅接受配置中的九个黄色格编号，失败时按配置次数重试。
- 任务阶段通过话题发布，异常时进入 `ABORTED`，成功结束时进入 `DONE`。

这些是代码中已经实现的机制，具体效果仍需通过实车或仿真实验测量。

