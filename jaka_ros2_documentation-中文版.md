# JAKA ROS 2 说明手册
## 介绍
### ROS 2 概述
- **ROS（机器人操作系统）** 是一组开源软件库和工具，为机器人应用的开发和运行提供了一个灵活的框架。  
- **ROS 2** 是 ROS 1 的升级版，在性能、安全性和实时性方面都有所改进，更适合科研和工业应用。  
- ROS 2 将代码组织成 **软件包（packages）**，并使用 `colcon` 在 **工作空间（workspace）** 中进行构建。每个软件包可能包含一个或多个 **节点（nodes）**，这些节点通过 **话题（topics）、服务（services）或动作（actions）** 进行通信。  
- **JAKA ROS 2 软件包** 主要使用：  
  - **话题（Topics）** 发送机器人状态数据。  
  - **动作（Actions）** 处理长时间运行的任务（例如运动执行）。  
  - **服务（Services）** 处理快速命令（例如启用机器人）。  

### 目的和范围
#### 目的  
本文件提供了安装和配置 **JAKA ROS 2 软件包** 的指南，并包含针对 **仿真** 和 **真实机器人操作** 的逐步教程。  

本手册适用于熟悉基本 **ROS 2 概念**、并希望将 JAKA 机器人集成到其应用中的开发者。

#### 范围 
##### JAKA 机器人型号  
当前版本的 **JAKA ROS 2 软件包** 支持所有官方发布的 JAKA 六自由度协作机器人型号，包括 `JAKA A 系列`、`JAKA C 系列`、`JAKA Pro 系列`、`JAKA S 系列`、`JAKA Zu 系列` 和 `JAKA MiniCobo`。


本手册中的所有配置文件、驱动程序和教程均适用于这些型号。  

##### 软件功能范围：  
- **MoveIt 集成**：运动规划与执行，包括碰撞检测、运动学计算和轨迹生成。  
- **RViz 可视化与仿真**：  
  - 实时可视化机器人模型、关节状态和规划轨迹。  
  - 虽然 RViz 主要是一个可视化工具，但本软件包提供了一个基本的 **类仿真环境**，用于预览规划的运动并与机器人的状态交互。  
- **Gazebo 仿真环境**：完整的机器人动态仿真，包括物理交互和真实世界动态。Gazebo 非常适合测试控制算法以及机器人在逼真的仿真环境中的动态响应。
- **控制驱动接口**：一个 ROS 2 驱动节点，用于向机器人控制器发送指令并接收状态反馈。  

### JAKA ROS 2 软件包结构
以下两张流程图分别展示了 JAKA ROS 1 和 JAKA ROS 2 软件包的结构。  

<figure id="figure-1-1">
  <img src="images/Figure 1-1: JAKA_ROS1_Package_Structure.png" alt="JAKA ROS1 Package Structure">
  <figcaption>
    <p align="center"><strong>图 1-1：JAKA ROS1 软件包结构</strong></p>
  </figcaption>
</figure>

<figure>
  <img src="images/Figure 1-2: JAKA_ROS2_Package_Structure.png" alt="JAKA ROS2 Package Structure">
</figure>
<div align="center">
  <h5 id="figure-1-2"><strong>图 1-2: JAKA ROS2 软件包结构</strong></h5>
</div>

在从 ROS 1（Catkin）迁移到 ROS 2（Colcon）的过程中，JAKA 机器人软件包的整体布局基本保持不变，但其构建和安装过程有所变化。 

以下是关键区别的概述，以及这些变化的重要性。

#### 与 ROS 1 的对比
* **ROS 1 (Catkin)**:
    - `build/`：存放 CMake 的中间编译文件。  
    - `devel/`：包含开发阶段的二进制文件、脚本及其他运行时资源。  
    - `install/`：通常用于可选的最终安装输出（在正常开发过程中较少使用）。  
* **ROS 2 (Colcon)**:
    - `build/`：仍然包含中间构建产物。  
    - `install/`：现在作为主要的运行时目录，取代了 `devel/` 的功能。  
    - `log/`：存储构建日志和调试信息。  

#### 变化原因
- **清晰性与模块化**：通过将构建产物（`build/`）与最终安装产物（`install/`）分开，ROS 2 更清晰地区分了运行时所需的文件与构建过程中产生的文件。  

- **软件包隔离**：Colcon 强制执行更一致的工作空间结构，减少依赖冲突，并简化软件包在其他系统上的部署。

### 支持的平台与系统要求
**JAKA ROS 2 软件包** 旨在无缝集成 JAKA 协作机器人至 ROS 2 生态系统中。

为了确保稳定性和性能，该软件包已在特定的软件和硬件配置上进行测试和验证，详情如下。  

#### 支持的 ROS 2 版本
- 目前，该软件包已在 **Ubuntu 22.04 (Jammy Jellyfish) + ROS 2 Humble** 上正式测试和支持。该软件包主要针对 **ROS 2 Humble** 进行测试，但也可能在 **ROS 2 Galactic** 或 **Iron** 上构建和运行。这种情况不保证完全兼容。  
- 本软件包设计用于与 **Gazebo Ignition Fortress** 配合使用，这是 **ROS 2 Humble** 推荐的仿真环境，提供稳定、高性能的基于物理的仿真。该软件包也已在 **Gazebo Classic** 上进行过测试，尽管功能上可用，但由于其已过时，不推荐在 ROS 2 Humble 上使用。用户可能会遇到不稳定和性能问题。
- 本软件包的未来版本可能会扩展对新版本 ROS 2 和 Gazebo 的支持，具体取决于它们的正式发布。

#### 支持的控制器版本
为确保 JAKA ROS 2 软件包的最佳性能和兼容性，我们已经为控制器设定了特定的要求。最低支持的控制器版本为 **1.7.1.46**，该版本提供了基本操作所需的核心功能。为了提高稳定性、增强功能并确保与 ROS 2 和 MoveIt 2 的无缝兼容，推荐的控制器版本为 **1.7.2.16**。我们建议用户升级至推荐版本，因为该版本经过充分测试，并且是当前可用的最新版本。

#### 最低硬件要求
推荐的最低系统要求如下表： 

| **组件**   | **推荐最低要求**   | **备注**           |
| -------- | ----------------- | ------------------- |
| **处理器 (CPU)**   | 64 位 Intel i5 / i7（或同等 AMD 处理器） | 推荐使用多核 CPU，以运行运动规划和实时传感器处理。           |
| **内存 (RAM)**     | 至少 8 GB（推荐 16 GB） | MoveIt 2 运动规划和 RViz 可视化需要较大的内存。              |
| **存储 (Storage)** | 至少 20 GB 可用空间（建议使用 SSD）  | 更快的存储可提高启动速度和数据处理能力。          |
| **GPU (可选)**     | 支持 CUDA 的 NVIDIA GPU  | 对于基本功能不是必需的，但对于高级感知应用（如基于 AI 的视觉处理、实时 SLAM）会有所帮助。 |


### ROS 2 基本命令
本节介绍在使用 JAKA ROS 2 软件包时常用的基础 ROS 2 命令。这些命令可用于构建、运行和调试 ROS 2 节点，管理主题、服务、参数以及启动文件。 

与 ROS 1 不同，ROS 2 采用更模块化和灵活的架构，使用 **Colcon** 进行工作空间构建，使用 **DDS（数据分发服务）** 进行通信，并用 `ros2` CLI 工具替代了 `rostopic`、`rosnode` 和 `rosservice` 等 ROS 1 命令。  

下表汇总了 ROS 2 的关键命令及其功能。  

| **命令**     | **说明**          |
| ------- | ------------- |
| `cd ~/<ros2_ws>`  | 进入 ROS 2 工作空间目录。     |
| `colcon build`    | 编译 ROS 2 软件包（类似于 ROS 1 中的 `catkin_make`）。       |
| `source ./install/setup.bash`     | 添加软件包到环境变量（类似于 ROS 1 中的 `source ./devel/setup.bash`）。 |
| `ros2 pkg list`                   | 查看所有可用的软件包。         |
| `ros2 pkg prefix <package-name>`  | 查找软件包的安装目录。      |
| `ros2 run <package-name> <executable-name>`    | 运行 ROS 2 节点。      |
| `ros2 node list`                               | 查看所有正在运行的节点。      |
| `ros2 node info <node-name>`                   | 查看指定节点的信息。          |
| `ros2 node kill <node-name>`                   | 停止运行中的节点。            |
| `ros2 topic --help`                            | 查看所有可用的话题操作。       |
| `ros2 topic list`                              | 查看所有活动中的话题。          |
| `ros2 topic echo /<topic_name>`                | 实时显示指定话题的消息内容。  |
| `ros2 topic info /<topic_name>`                | 查看话题的信息（消息类型、发布者、订阅者等）。     |
| `ros2 topic type /<topic_name>`                | 查看话题的消息格式。         |
| `ros2 interface show <msg_type>`               | 显示特定消息类型的详细信息（类似于 ROS 1 中的 `rosmsg show`）。 |
| `ros2 service --help`                          | 查看所有可用的服务操作。                   |
| `ros2 service list`                            | 查看活动中的服务列表。                     |
| `ros2 service call <service-name> <service-type> {argument}` | 手动调用服务。              |
| `ros2 launch <package_name> <file.launch.py>`                | 运行启动文件。              |
| `ros2 param list`                                            | 查看某个节点的可用参数。     |
| `ros2 param get <node-name> <param-name>`                    | 获取某个参数的值。           |
| `ros2 param set <node-name> <param-name value>`              | 设置参数的值。        |
| `ros2 bag record /<topic_name>`                              | 记录某个话题的消息到 ROS 2 bag 文件中。 |
| `ros2 bag play <bag_file>`                                   | 重新播放已记录的 bag 文件消息。       |
| `ros2 bag info <bag_file>`                                   | 查看已记录 bag 文件的详细信息。       |
| `ros2 doctor`                                                | 诊断并检查 ROS 2 运行环境。      |

## 安装与设置
### 前置要求
在安装和使用 **JAKA ROS 2 软件包** 之前，请确保您的系统满足必要的软件要求。 

这包括配置兼容的操作系统、安装 ROS 2 及其依赖项，并设置运动规划和执行所需的额外工具。  

以下部分将提供逐步指南，帮助您准备开发环境。  

#### 操作系统
为了确保兼容性和稳定性，**JAKA ROS 2 软件包** 需要：

- **Ubuntu 22.04 (Jammy)** 操作系统，**x86_64** 架构。  
- **ROS 2 Humble** 作为中间件框架。  

#### ROS 2 安装
对于运行 **Ubuntu Jammy (22.04) x86_64** 架构的系统，请参考官方指南安装 ROS 2 Humble：[ROS 2 Humble 安装](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)。

为了方便起见，以下是安装的简要步骤。

**(1) 设置 Locale**  
确保系统支持 UTF-8 语言环境：

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

**(2) 配置 ROS 2 软件包源**  **  

- 首先，启用 Ubuntu Universe 仓库：  

  ```bash
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  ```

- 然后，添加 ROS 2 GPG 密钥：  

  ```bash
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```

- 最后，将 ROS 2 仓库添加到系统软件源列表：

  ```bash
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```

**(3) 安装 ROS 2 Humble**  
在安装之前，请更新并升级系统：

```bash
sudo apt update
sudo apt upgrade
```

选择适合的安装类型：

- **桌面版安装（推荐）**： 包含 ROS、RViz、示例和教程。  

  ```bash
  sudo apt install ros-humble-desktop
  ```

- **基础版安装（最小化设置）**： 仅包含核心 ROS 2 通信工具，不包括 GUI 工具。

  ```bash
  sudo apt install ros-humble-ros-base
  ```

- **开发工具（用于构建软件包）**： 如果需要开发或编译 ROS 2 软件包，可额外安装此工具。  

  ```bash
  sudo apt install ros-dev-tools
  ```

**(4) 环境配置**  
每次打开新终端时，都需要手动加载 ROS 2 环境：  

```bash
source /opt/ros/humble/setup.bash
```

如果希望每次打开终端时自动加载 ROS 2，可将其添加到 .bashrc 文件：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **注意:** 如果使用不同的 shell（例如 zsh），请将 .bash 替换为 .zsh 或 .sh。

#### 额外依赖项
**MoveIt 2 安装**

- MoveIt 2 是运动规划和执行所必需的组件。建议使用 **MoveIt 2 版本 2.2+**，确保在使用 JAKA ROS 2 软件包之前已正确安装 MoveIt 2。
- 参考官方 MoveIt 2 文档进行安装：二进制安装（官方 Debian 软件包 via apt）： [MoveIt 2 二进制安装](https://moveit.ai/install-moveit2/binary/).   

```bash
    sudo apt update 
    sudo apt install ros-humble-moveit
    source /opt/ros/humble/setup.bash
```

**Gazebo Fortress 安装**
- 本软件包支持 **Gazebo Fortress** 与 ROS 2 Humble 的仿真。确保通过安装 ros-gz bridge 包正确设置 ROS 2 与 Gazebo Fortress 的集成。
- 要安装 Gazebo Fortress，请按照官方的 Ignition Gazebo 安装指南进行操作：[Gazebo Fortress 二进制安装](https://gazebosim.org/docs/fortress/install_ubuntu/)。

**(1) 安装必要的工具：**  
```bash
sudo apt-get update
sudo apt-get install lsb-release gnupg
```  

**(2) 安装 Ignition Fortress：**  
```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```  

**(3) 安装 ROS 2 集成（ros-gz Bridge）：** 允许 ROS 2 Humble 与 Gazebo Fortress 之间的通信。  
```bash
sudo apt install -y ros-humble-ros-gz
```

**(4) 安装 ign_ros2_control 软件包：** 提供 **IgnitionSystem** 插件。  
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ign-ros2-control
```  

**(5) Ubuntu 虚拟机用户的额外配置**  
如果你在 **虚拟机** 中运行 Gazebo Fortress，可能会遇到 **闪烁网格** 或 **空白渲染窗口** 的问题，这是由于硬件加速有限导致的。为了解决这个问题，可以通过添加以下内容强制使用软件渲染：  
```bash
echo 'export LIBGL_ALWAYS_SOFTWARE=true' >> ~/.bashrc
source ~/.bashrc
```  

### 安装 JAKA ROS 2 软件包 
**(1) 从 GitHub 克隆或下载发布版本**  
要获取 JAKA ROS 2 软件包，可以选择以下方式：

- 从 GitHub 克隆仓库：  
a. 使用 HTTPS 克隆： 

  ```bash
  git clone https://github.com/JAKARobotics/jaka_ros2.git
  ```

b. 使用 SSH 克隆： 

  ```bash
  git clone git@github.com:JAKARobotics/jaka_ros2.git
  ```

- 从官方来源下载最新发布版本并解压软件包。  

**(2) 使用 colcon 进行构建**  
获取软件包后，进入工作空间并使用 colcon 进行构建：

```bash
cd <path-to-where-the-repository-is-cloned-or-extracted>/jaka_ros2
colcon build --symlink-install
```

**(3) 终端环境设置**  
构建完成后，需对环境进行设置以使用该软件包： 

```bash
source <path-to-where-the-repository-is-cloned-or-extracted>/jaka_ros2/install/setup.bash
```

如果希望在每次打开终端时自动加载：

```bash
echo "source <path-to-where-the-repository-is-cloned-or-extracted>/jaka_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

至此，JAKA ROS 2 软件包的安装与配置已完成。

您现在可以开始使用该软件包进行机器人控制和运动规划。

### 切换到不同的 SDK 版本
**JAKA ROS 2** 软件包目前基于 **JAKA SDK 版本 2.2.2**，这是最新的可用版本。

如果您需要使用早期的 SDK 版本，如 **版本 2.1.14**，请按照以下步骤调整您的设置。

#### 切换 SDK 版本的步骤：
**(1) 下载所需的 SDK 软件包** **  
访问 JAKA 官方网站下载您希望使用的 SDK 版本：[JAKA 下载](https://www.jaka.com/download)。

**(2) 替换 `libjakaAPI.so` 文件**  
a. 进入您已安装的 JAKA ROS 2 工作空间中的 `jaka_driver` 和 `jaka_planner` 包的 `lib` 目录。
b. 使用从下载的 SDK 中提供的版本替换 `libjakaAPI.so` 文件。 
c. 对于 **版本 2.1.14**，`libjakaAPI.so` 文件可以在解压后的 SDK 文件夹中的以下目录找到：`20241018100240A001/Linux/c&c++/x86_64-linux-gnu/shared`

**(3) 替换头文件**  
a. 进入您已安装的 JAKA ROS 2 工作空间中的以下 `include` 目录：   

- `jaka_driver/include/jaka_driver`  
- `jaka_planner/include/jaka_planner`  

b. 使用从下载的 SDK 中获得的对应头文件替换以下头文件：  

- `JAKAZuRobot.h`  
- `jkerr.h`  
- `jktypes.h`  

c. 对于 **版本 2.1.14**，这些文件可以在解压后的 SDK 文件夹中的以下目录找到：`20241018100240A001/Linux/c&c++/inc_of_c++`

**(4) 无需修改 `conversion.h`**  
`jaka_driver/include/jaka_driver` 中的 `conversion.h` 文件不是 JAKA SDK 的一部分，因此不需要更改，您可以保持该文件不变。

按照这些步骤，您将成功地在 JAKA ROS 2 工作空间中切换到不同的 SDK 版本（如 2.1.14），确保与所选的 SDK 版本兼容。

## JAKA ROS 2 工作空间概述
本章提供了 JAKA ROS 2 工作空间的概述，详细介绍了其关键组件、节点、可执行文件、主题、服务和动作。

### JAKA ROS 2 包概述
如 [图 1-2](#图-1-2-jaka-ros2-软件包结构) 所示，JAKA ROS 2 工作空间由多个包组成，每个包在JAKA机器人与ROS 2应用方面都有不同作用。

- **jaka_driver**: 启动控制驱动包，包括源代码、SDK 库和低级驱动接口的启动文件。
- **jaka_planner**: 提供使用 MoveIt 2 进行运动规划的功能，包括与规划相关的配置和启动文件，以及基于动作的轨迹执行。
- **jaka_msgs**: 包含在其他包中使用的自定义消息和服务定义。
- **jaka_description**: 存放 URDF、网格和描述机器人模型的配置文件。
- **jaka_`<robot_model>`_moveit_config**: 针对每个支持的 JAKA机器人模型（`<robot_model>`：`zu3`、`s5`、`a12`、`minicobo` 等）单独的 MoveIt 包，其中包含 MoveIt 2 配置文件、RViz 可视化设置和 YAML 参数文件。

### JAKA 驱动包 (`jaka_driver`)
`jaka_driver` 包负责低级别的机器人通信和控制。它提供了用于运动控制、配置和状态报告的 ROS 2 服务和主题。  

它包含以下关键组件。

#### 节点和可执行文件
下表提供了 `jaka_driver` 包中可执行文件的概览，包含它们关联的节点及功能。

| **可执行文件**      | **节点名称**             | **目的**                                                         |
| --------------- | -------------------- | ---------------------------------------------------- |
| jaka_driver.cpp | *jaka_driver*        | 主要的驱动节点，用于初始化机器人，并处理 ROS 2 主题和服务调用，负责状态管理、I/O 和运动控制。 |
| client.cpp      | *linear_move_client* | 一个 ROS 2 客户端节点，向 `/jaka_driver/linear_move` 服务发送服务请求，命令机器人根据用户指定的姿势和运动参数进行线性运动。 |
| sdk_test.cpp    | *moveit_server*      | 通过连接机器人、启用机器人并获取关节位置来测试 JAKA 机器人 SDK，以验证通信和状态信息。 |
| servoj_demo.cpp | *client_test*        | 使用 `/jaka_driver/servo_move_enable` 服务启用机器人的伺服模式，将机器人切换到伺服模式。启用伺服模式后，使用 `/jaka_driver/servo_j` 服务逐步移动机器人的关节到预定位置，从而创建连续的运动效果。 |

#### 服务
`jaka_driver` 节点提供了各种服务，按以下类别进行分类：

##### 运动控制
- **/jaka_driver/linear_move** - 在用户坐标系中执行线性运动。
- **/jaka_driver/joint_move** - 在关节坐标系中执行轴向运动。
- **/jaka_driver/jog** - 在关节坐标系、用户坐标系或工具坐标系中启用连续运动控制（自由驱动模式）。
- **/jaka_driver/servo_move_enable** - 启用伺服位置控制模式。
- **/jaka_driver/servo_p** - 在伺服模式下控制笛卡尔坐标系中的运动。
- **/jaka_driver/servo_j** - 在伺服模式下控制关节坐标系中的运动。
- **/jaka_driver/stop_move** - 停止机器人的运动。

##### 参数配置
- **/jaka_driver/set_toolframe** - 配置 TCP（工具中心点）参数。
- **/jaka_driver/set_userframe** - 配置用户坐标系参数。
- **/jaka_driver/set_payload** - 设置机器人臂的载荷参数。
- **/jaka_driver/drag_move** - 启用或禁用自由驱动模式。
- **/jaka_driver/set_collisionlevel** - 调整碰撞检测的灵敏度。

##### I/O 和运动学
- **/jaka_driver/set_io** - 配置控制面板、工具或扩展接口上的数字或模拟 I/O 值。
- **/jaka_driver/get_io** - 从控制面板、工具或扩展接口获取数字或模拟 I/O 值。
- **/jaka_driver/get_fk** - 计算正向运动学。
- **/jaka_driver/get_ik** - 计算逆向运动学。

#### 主题
`jaka_driver` 节点通过以下主题发布实时的机器人状态更新：

- **/jaka_driver/tool_position** - 末端执行器的位置和方向。
- **/jaka_driver/joint_position** - 关节位置的信息。
- **/jaka_driver/robot_states** - 机器人整体状态和事件信息。

### JAKA 规划器 (`jaka_planner`)
`jaka_planner` 包提供了使用 MoveIt 2 进行轨迹规划和执行的功能。

它包含以下关键组件。

#### 节点和可执行文件
下表提供了 `jaka_planner` 包中可执行文件的概览，包含它们关联的节点及其功能。

| **可执行文件**        | **节点名称**        | **目的**                                              |
| ----------------- | --------------- | ----------------------------------------------- |
| moveit_server.cpp | *moveit_server* | 处理轨迹执行和机器人状态监控。它连接到机器人，启用伺服模式，并处理运动规划和执行的轨迹目标。 |
| moveit_test.cpp   | *jaka_planner*  | 用于 MoveIt 2 运动规划的测试节点。它演示了关节空间和笛卡尔空间中的运动规划，并使用 move_group 接口执行规划的轨迹。 |

#### 主题
**/joint_states** - 发布机器人关节状态，用于 MoveIt 2 和 RViz 可视化。

#### 动作
`jaka_planner` 包提供了一个动作服务器，用于执行规划的轨迹：

- **动作名称**： `/jaka_<robot_model>_controller/follow_joint_trajectory` (`<robot_model>`：`zu3`、`s5`、`a12`、`minicobo` 等)
- **消息类型**： `control_msgs::action::FollowJointTrajectory`
- **功能**：
  - 接收并执行关节轨迹目标。
  - 监控执行进度。
  - 提供连续状态更新并确认完成。

## 入门指南（教程）
### 基础教程：使用 jaka_driver 包控制 JAKA 机器人
`jaka_driver` 包是一个控制驱动程序，通过 ROS 2 服务和主题与 JAKA 机器人进行通信。

本节提供了示例服务调用和可执行文件运行，这些调用请求各种服务或 SDK 功能，以向机器人发送基本的控制命令并检查其状态。

#### 启动 JAKA ROS 2 驱动程序
在执行机器人命令之前，必须启动 `jaka_driver` 节点。

使用以下命令启动驱动程序并与机器人建立连接：

```bash
ros2 launch jaka_driver robot_start.launch.py ip:=<robot_ip>
```
> **注意:** 请将 <robot_ip> 替换为实际的机器人 IP 地址。此参数是成功通信所必需的。

<figure id="figure-4-1">
  <img src="images/Figure 4-1: Launch JAKA Driver Server.png" alt="Launch JAKA Driver Serve">
  <figcaption>
    <p align="center"><strong>图 4-1： 启动 JAKA 驱动服务器命令输出</strong></p>
  </figcaption>
</figure>

#### 示例服务命令
以下示例演示了如何使用 `jaka_driver` 包向 JAKA 机器人发送控制命令。

**(1)	执行关节运动**  
要在关节坐标系中移动机器人，调用 `/jaka_driver/joint_move` 服务，并传递适当的参数，如下所示：

```bash
ros2 service call /jaka_driver/joint_move jaka_msgs/srv/Move "{
  pose: [0, 1.57, -1.57, 1.57, 1.57, 0],
  has_ref: false,
  ref_joint: [0],
  mvvelo: 0.5,
  mvacc: 0.5,
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"
```
> **注意：** 默认情况下，关节运动接口是阻塞的。如需使用非阻塞接口，请修改 **jaka_driver.cpp** 中的相应参数。

  <figure id="figure-4-2">
    <img src="images/Figure 4-2: Joint Motion Service.png" alt="Joint Motion Service">
    <figcaption>
      <p align="center"><strong>图 4-2：关节运动服务命令输出 </strong></p>
    </figcaption>
  </figure>  

  <figure id="figure-4-3">
    <img src="images/Figure 4-3: Joint Motion Service Execution.png" alt="Joint Motion Service Execution">  
    <figcaption>
      <p align="center"><strong>图 4-3：关节运动服务执行</strong></p>
    </figcapti

**(2) 执行线性运动**  
要命令机器人在用户坐标系中执行线性运动，请使用 `/jaka_driver/linear_move` 服务，并传递所需参数，如下所示：

```bash
ros2 service call /jaka_driver/linear_move jaka_msgs/srv/Move "{
  pose: [111.126, 282.111, 271.55, 3.142, 0, -0.698],
  has_ref: false,
  ref_joint: [0],
  mvvelo: 100,
  mvacc: 100,
  mvtime: 0.0,
  mvradii: 0.0,
  coord_mode: 0,
  index: 0
}"
```

> **重要：** 本示例中的姿态参数仅供参考。确保姿态值在机器人工作空间内，并且不会导致奇异情况或超出运动限制。


  <figure id="figure-4-4">
    <img src="images/Figure 4-4: Linear Motion Service.png" alt="Linear Motion Service">
    <figcaption>
      <p align="center"><strong>图 4-4：线性运动服务命令输出</strong></p>
    </figcaption>
  </figure>

  <figure id="figure-4-5">
    <img src="images/Figure 4-5: Linear Motion Service Execution.png" alt="Linear Motion Service Execution">
    <figcaption>
      <p align="center"><strong>图 4-5：线性运动服务执行</strong></p>
    </figcaption>
  </figure>

**(3)	计算正向运动学**  
要计算给定关节位置下机器人的正向运动学，调用 `/jaka_driver/get_fk` 服务，并传递所需的输入参数，如下所示：
```bash
ros2 service call /jaka_driver/get_fk jaka_msgs/srv/GetFK "{
joint: [0, 1.57, -1.57, 1.57, 1.57, 0]
}"
```

  <figure id="figure-4-6">
    <img src="images/Figure 4-6: Forward Kinematics Service.png" alt="Forward Kinematics Service">
    <figcaption>
      <p align="center"><strong>图 4-6：正向运动学服务命令输出</strong></p>
    </figcaption>
  </figure>

**(4)	计算逆向运动学**  
要确定实现特定笛卡尔姿态所需的关节配置，请使用 `/jaka_driver/get_ik` 服务，并传递适当的输入参数，如下所示：

```bash
ros2 service call /jaka_driver/get_ik jaka_msgs/srv/GetIK "{
ref_joint: [0, 1.57, -1.57, 1.57, 1.57, 0],
cartesian_pose: [130.7, 116, 291, 3.13, 0, -1.5707]
}"
```

  <figure id="figure-4-7">
    <img src="images/Figure 4-7: Inverse Kinematics Service.png" alt="Inverse Kinematics Service">
    <figcaption>
      <p align="center"><strong>图 4-7：逆向运动学服务命令输出</strong></p>
    </figcaption>
  </figure>

#### 在 JAKA 驱动包中运行可执行文件
除了 `jaka_driver` 节点外，JAKA 驱动包还包括其他可执行文件，用于基本测试 `jaka_driver` 包中某些服务和 SDK 功能。这些可执行文件允许您以不同的方式与机器人交互、测试特定的服务、并验证机器人的功能。  
以下是运行这些可执行文件的必要步骤和示例。
**(1) R运行 Servo J 可执行文件**  
`servoj_demo` 可执行文件用于通过启用机器人伺服模式并命令机器人逐步移动到特定的关节位置来测试机器人的伺服模式。此可执行文件测试了 `/jaka_driver/servo_move_enable` 和 `/jaka_driver/servo_j` 服务，这些服务在伺服模式下控制机器人的关节运动。

要运行此可执行文件，首先确保 `jaka_driver` 节点正在运行。使用以下命令启动驱动程序并建立与机器人的连接：
```bash
ros2 launch jaka_driver robot_start.launch.py ip:=<robot_ip>
```
> **注意：**
将 `<robot_ip>` 替换为您的机器人的实际 IP 地址。  


然后，使用以下命令启动可执行文件：
```bash
ros2 run jaka_driver servoj_demo
```

这将启用伺服模式并开始执行关节运动。

伺服运动将逐步移动机器人到指定的关节位置，演示其基本控制功能。

  <figure id="figure-4-8">
    <img src="images/Figure 4-8: Servoj Demo Executable.png" alt="Servoj Demo Executable">
    <figcaption>
      <p align="center"><strong>图 4-8：Servoj 演示可执行命令输出 </strong></p>
    </figcaption>
  </figure>

  <figure id="figure-4-9">
    <img src="images/Figure 4-9: ServoJ Demo Execution.png" alt="ServoJ Demo Execution">
    <figcaption>
      <p align="center"><strong>图 4-9：ServoJ 演示执行</strong></p>
    </figcaption>
  </figure>

**(2) 运行客户端可执行文件**  
`client` 可执行文件是一个服务客户端节点，用于测试 `/jaka_driver/linear_move` 服务。此可执行文件向 `linear_move` 服务发送请求，将机器人移动到其工作空间中的特定笛卡尔位置。  

使用客户端之前，首先确保 `jaka_driver` 节点正在运行。使用以下命令启动驱动程序并建立与机器人的连接：

```bash
ros2 launch jaka_driver robot_start.launch.py ip:=<robot_ip>
```
> **注意：** 
将 `<robot_ip>` 替换为您的机器人的实际 IP 地址。


然后，在另一个终端中执行以下命令：
```bash
ros2 run jaka_driver client <x_position>, <y_position>, <z_position>, <rx>, <ry>, <rz>
```
> **注意：**
将 `<x_position>`、`<y_position>`、`<z_position>`、`<rx>`、`<ry>`、`<rz>` 替换为您希望机器人目标姿势的笛卡尔坐标和方向。确保这些值在机器人的工作空间内，并避免奇异点。  

示例：
```bash
ros2 run jaka_driver client 111.126, 282.111, 271.55, 3.142, 0, -0.698
```

  <figure id="figure-4-10">
    <img src="images/Figure 4-10: Client Executable.png" alt="Client Executable">
    <figcaption>
      <p align="center"><strong>图 4-10：客户端可执行命令输出</strong></p>
    </figcaption>
  </figure>

  <figure id="figure-4-11">
    <img src="images/Figure 4-11: Client Execution.png" alt="Client Execution">
    <figcaption>
      <p align="center"><strong>图 4-11：客户端执行</strong></p>
    </figcaption>
  </figure>

**(3) 运行 SDK 测试可执行文件**  
`sdk_test` 可执行文件连接到指定 IP 地址的机器人，并通过启用机器人并检索当前关节位置来验证与机器人的通信。这是一个基本的验证步骤，用于确保机器人已开机且连接正常。  

该可执行文件使用 SDK 访问机器人的当前关节位置并显示检索到的值。与前两个可执行文件不同，`sdk_test` 不需要 `jaka_driver` 节点运行，因为它会自动开启机器人并启用它。  

要运行 SDK 测试，只需执行以下命令：

```bash
ros2 run jaka_driver sdk_test --ros-args -p ip:=<robot_ip>
```
> **注意：** 
将 `<robot_ip>` 替换为您的机器人的实际 IP 地址。


  <figure id="figure-4-12">
    <img src="images/Figure 4-12: SDK Test Executable.png" alt="SDK Test Executable">
    <figcaption>
      <p align="center"><strong>图 4-12：SDK 测试可执行命令输出</strong></p>
    </figcaption>
  </figure>

### MoveIt 2 教程：规划与执行
MoveIt 2 是一个强大的运动规划框架，能够为机器人臂提供轨迹规划和执行。尽管更新后的 `jaka_ros2` 软件包现在支持 **Gazebo 仿真** 进行实时轨迹执行，但仍提供基于 **RViz 的仿真模式**，适用于由于系统限制、硬件约束或其他需求无法使用 Gazebo 的情况。

为了启用该 RViz 仿真模式，需要对默认的 MoveIt 2 包安装中的 `launches.py` 文件进行修改。`jaka_ros2` 包提供了这个文件的调整版本，用户可以将其替换到系统中以实现所需功能。

对 `launches.py` 文件的关键修改是添加了 `use_rviz_sim` 参数。当 `use_rviz_sim` 设置为 `true` 时，将启用仿真模式。默认情况下，`use_rviz_sim` 设置为 `false`。

#### 设置 MoveIt 2 与 JAKA 机器人（RViz 仿真）
(1) 确保 **MoveIt 2** 和 `jaka_ros2` 包已正确安装。  
(2) 将默认的 `launches.py` 文件替换为 `jaka_ros2` 包中提供的修改版本（[launches.py](./launches.py)）。您可以使用 `find` 命令定位 MoveIt 2 安装目录中的 `launches.py` 文件，如下所示。

  <figure id="figure-4-13">
    <img src="images/Figure 4-13: Locate launches.py.png" alt="Locate launches.py">
    <figcaption>
      <p align="center"><strong>图 4-13：定位 launches.py 命令输出</strong></p>
    </figcaption>
  </figure>

(3) 使用以下命令在 RViz 仿真模式下启动 MoveIt 2：  

```bash
ros2 launch jaka_<robot_model>_moveit_config demo.launch.py use_rviz_sim:=true
```

> **注意：** 
将 `<robot_model>` 替换为相应的 JAKA 机器人模型名称（例如， `zu3`、`s5`、`a12`、`minicobo` 等）。


  <figure id="figure-4-14">
    <img src="images/Figure 4-14: Launch Demo Moveit Config in Simulation Mode.png" alt="Launch Demo Moveit Config in Simulation Mode ">
    <figcaption>
      <p align="center"><strong>图 4-14：在仿真模式下启动演示 Moveit 配置命令输出</strong></p>
    </figcaption>
  </figure>

#### 规划与执行轨迹
(1) 启动 RViz 后，机器人模型将显示在环境中。  
(2) 将 **交互标记**（表示为机器人末端执行器的球体）移动到所需的目标位置，或从 RViz 界面的 **"Goal state"** 中选择目标姿态。 
(3) 点击 **"Plan & Execute"** 生成并可视化机器人的轨迹。

  <figure id="figure-4-15">
    <img src="images/Figure 4-15: RViz Simulation Mode Execution.png" alt="RViz Simulation Mode Execution">
    <figcaption>
      <p align="center"><strong>图 4-15：RViz 仿真模式执行</strong></p>
    </figcaption>
  </figure>

这种方法允许用户在 RViz 环境中进行轨迹规划和执行，而无需物理 JAKA 机器人或 3D 仿真软件工具，从而确保在各种开发环境中具有更大的灵活性和更广泛的可访问性。

### Gazebo 仿真教程：在 Gazebo 中进行实时轨迹执行  
Gazebo 是一个强大的仿真环境，提供逼真的物理和可视化效果，适用于机器人应用。在本节中，我们演示如何使用 Gazebo 与 `jaka_ros2` 包结合，在仿真环境中模拟实时轨迹规划和执行。  
与[上一个节](#moveit-2-教程规划与执行) 中描述的 RViz 仿真类似，本教程使用了自定义的启动文件（即修改过的 `launches.py`）。对 [launches.py](./launches.py) 所做的主要修改是增加了用于独立启动 Gazebo 进行模型可视化的功能，并启动一个集成的 Gazebo 仿真环境与 RViz 配合使用。

#### 使用 JAKA ROS2 设置 Gazebo 仿真

**(1) 确保已安装所需的软件包：**  
在继续之前，确保 **Gazebo Ignition Fortress** 和 `jaka_ros2` 包已经正确安装。

**(2) 更新自定义启动文件：**  
将默认的 `launches.py` 文件替换为 `jaka_ros2` 包提供的自定义版本（详情请参阅 [上一个节](#设置-moveit-2-与-jaka-机器人rviz-仿真)）。

**(3) 仅启动 Gazebo 进行模型可视化：**  
要在 Gazebo 中可视化 JAKA 机器人模型，使用自定义的 `gazebo.launch.py` 文件运行以下命令：  
   ```bash
   ros2 launch jaka_<robot_model>_moveit_config gazebo.launch.py
   ```  
  <figure id="figure-4-16">
    <img src="images/Figure 4-16: Launch Gazebo Simulation Independently.png" alt="Independent Gazebo simulation Execution" width="1200">
    <figcaption>
      <p align="center"><strong>图 4-16: 独立的 Gazebo 仿真执行</strong></p>
    </figcaption>
  </figure>

> **注意：** 替换 `<robot_model>` 为适当的 JAKA 机器人模型名称（例如：zu3、s5、a12、minicobo 等）。

此命令启动 Gazebo Fortress，加载默认世界（一个仅包含地面平面和光源的空白世界），并根据发布的机器人描述生成机器人模型。机器人将出现在 Gazebo GUI 中以供检查。

**(4) 启动 Gazebo 与 RViz 进行轨迹规划与执行：**  
为了在仿真环境中启用实时轨迹规划与执行，运行 `demo_gazebo.launch.py` 文件，将 Gazebo 与 RViz 集成：  
   ```bash
   ros2 launch jaka_<robot_model>_moveit_config demo_gazebo.launch.py
   ```  

> **注意：** 替换 `<robot_model>` 为适当的 JAKA 机器人模型名称（例如：zu3、s5、a12、minicobo 等）。

#### 在 Gazebo 中演示轨迹执行
**(1) 启动演示环境：**  
   按照[上述步](#使用-jaka-ros2-设置-gazebo-仿真)的步骤操作后，机器人模型将同时出现在 **Gazebo** 和 **RViz** 环境中。

**(2) 设置目标姿态：**  
   - 在 RViz 中通过移动机器人末端执行器上的 **交互标记** 到期望的位置。  
   - 或者，从 RViz 界面选择一个 **目标状态**。

**(3) 规划并执行轨迹：**  
   - 在 RViz 中点击 **"Plan & Execute"** 生成并可视化机器人的轨迹。  
   - 机器人在 **Gazebo** 中应该能够实时执行规划的轨迹。 

  <figure id="figure-4-17">
    <img src="images/Figure 4-17: RViz for Demo Gazebo Simulation Execution.png" alt="RViz for Demo Gazebo Simulation Execution">
    <figcaption>
      <p align="center"><strong>图 4-17： 演示 RViz  为 Gazebo 仿真执行</strong></p>
    </figcaption>
  </figure>

  <figure id="figure-4-18">
    <img src="images/Figure 4-18: Demo Gazebo Simulation with RViz Execution.png" alt="Demo Gazebo simulation with RViz Execution" width="1200">
    <figcaption>
      <p align="center"><strong>图 4-18： Gazebo 仿真与 RViz 演示执行</strong></p>
    </figcaption>
  </figure>

该过程允许用户在 Gazebo 仿真环境中测试运动规划和执行，而无需实际的机器人。

### 真实机器人教程：控制真实的 JAKA 机器人
#### 安全预防措施与机器人设置
在控制真实的 JAKA 机器人之前，确保正确的设置和安全措施至关重要，以防止碰撞、设备损坏或潜在的危险。

本节概述了关键的安全注意事项、负载配置和操作最佳实践。

**（1）环境与操作安全**

- **工作区域清理：** 确保机器人操作过程中工作区域没有障碍物和人员。如果需要，可以定义并执行限制区域。
- **电力与网络稳定性：** 使用稳定的电源，并确认控制计算机与机器人之间的网络连接，以防止意外中断。
- **紧急停止按钮访问：** 在操作前熟悉机器人紧急停止按钮，并确认其功能正常。

**（2）安全操作区与运动限制配置**

- **避免碰撞：** 在 JAKA 应用中配置安全区边界，以限制机器人在指定的安全区域内移动。
- **监控关节限制：** 确保规划的动作不会超过机器人关节的活动范围，以防止过度伸展。

**（3）负载配置**  
当安装外部工具，如抓手或其他末端执行器时，必须更新负载设置，以确保准确的运动控制并避免过度关节应力。

- **设置负载参数：** 使用 `/jaka_driver/set_payload` 服务或在 JAKA 应用中调整机器人负载配置，基于附加工具的重量和重心位置。
- **验证扭矩限制：** 过重的负载可能会导致关节超负荷。确保不会超过机器人的最大负载限制。
- **工具中心点 (TCP) 校准：** 定义准确的 TCP 以提高运动精度，防止工具偏移或碰撞。

**（4）运动规划与碰撞检测**

- **在 RViz 中模拟轨迹：** 在对真实机器人执行运动之前，首先在 RViz 中进行规划和验证，检查潜在的碰撞、奇异性错误或超范围运动。
- **设置安全运动参数：** 使用 MoveIt 2 配置文件或 RViz 界面为每个机器人模型包定义适当的速度和加速度限制，以确保安全执行。

#### 启动 `jaka_planner` 包
本节描述了如何启动 MoveIt 2 服务器和 RViz，以便使用真实的 JAKA 机器人进行轨迹规划和执行。

##### 前提条件
在启动 `jaka_planner` MoveIt 2 服务器之前，确保 `jaka_driver` 服务器**未运行**，因为 MoveIt 2 和 JAKA ROS 驱动程序不能同时运行。

如果 `jaka_driver` 正在运行，请先停止它再继续。

##### 启动 MoveIt 2 服务器
要启动 MoveIt 2 服务器，请打开一个终端并执行以下命令，将 `<robot_ip>` 替换为机器人的实际 IP 地址，将 `<robot_model>` 替换为您正在使用的 JAKA 机器人型号（例如，`zu3`、`s5`、`a12`、`minicobo` 等）：

```bash
ros2 launch jaka_planner moveit_server.launch.py ip:=<robot_ip> model:=<robot_model>
```

<figure id="figure-4-19">
  <img src="images/Figure 4-19: Launch MoveIt 2 Server.png" alt="Launch MoveIt 2 Server">
  <figcaption>
    <p align="center"><strong>图 4-19：启动 MoveIt 2 服务器命令输出</strong></p>
  </figcaption>    
</figure>

##### 在 RViz 中启动 MoveIt 2 客户端
在一个新终端中，使用以下命令启动 MoveIt 2 RViz 界面，将 `<robot_model>` 替换为您之前使用的相应 JAKA 机器人型号：
```bash
ros2 launch jaka_<robot_model>_moveit_config demo.launch.py
```
<figure id="figure-4-20">
  <img src="images/Figure 4-20: Launch Demo Moveit Config.png" alt="Launch Demo Moveit Config">
  <figcaption>
    <p align="center"><strong>图 4-20：启动示例 Moveit 配置命令输出</strong></p>
  </figcaption> 
</figure>

启动后，RViz 界面将打开，显示机器人模型。可视化应反映物理机器人的当前实际位置和方向。

<figure id="figure-4-21">
  <img src="images/Figure 4-21: RViz Real Robot Vizualization.png" alt="RViz Real Robot Vizualization">
  <figcaption>
    <p align="center"><strong>图 4-21：RViz 实际机器人可视化</strong></p>
  </figcaption>   
</figure>

<figure id="figure-4-22">
  <img src="images/Figure 4-22: JAKA App Real Robot Vizualization.png" alt="JAKA App Real Robot Vizualization">
  <figcaption>
    <p align="center"><strong>图 4-22：JAKA 应用程序实际机器人可视化</strong></p>
  </figcaption> 
</figure>

> **注意：** 
**use_rviz_sim** 参数的默认值为 **false**，因此在使用 MoveIt 2 和 RViz 控制真实机器人时，无需显式指定 `use_rviz_sim:=false`，除非您希望确保清晰。


#### 在 RViz 中使用真实机器人进行轨迹规划和执行
一旦 MoveIt 2 服务器和 RViz 界面启动，您可以直接从 RViz 进行轨迹规划和执行。在 RViz 中，您可以使用 **交互标记** 来规划和执行 JAKA 机器人的运动。  

在定义目标位置时，有两种执行方法：

**（1）规划并执行（不推荐用于初步测试）**

- 将 **交互标记** 拖动到末端执行器上的目标位置，或者从 RViz 界面的 **"Goal state"** 中选择目标姿势。
- 点击 **"Plan & Execute"** 将生成轨迹并立即发送给机器人执行。
- 这种方法会实时执行运动，而没有提前进行可视化。如果目标位置无法到达或会发生碰撞，可能会导致意外的运动。
- 只有在您**确定**目标位置的轨迹是安全的时，才使用此方法。

**（2） 推荐方法 – 先规划，再执行**

- 将 **交互标记** 拖动到末端执行器上的目标位置，或者从 RViz 界面的 **"Goal state"** 中选择目标姿势。
- 点击 **"Plan"** 在 RViz 中生成并可视化轨迹，然后再执行。
- 如果轨迹是 **安全且可行的**，点击 **"Execute"** 将运动命令发送给机器人。

<figure id="figure-4-23">
  <img src="images/Figure 4-23: RViz Real Robot Trajectory Planning and Execution.png" alt="RViz Real Robot Trajectory Planning and Execution">
  <figcaption>
    <p align="center"><strong>图 4-23：RViz 真实机器人轨迹规划和执行</strong></p>
  </figcaption>   
</figure>

<figure id="figure-4-24">
  <img src="images/Figure 4-24: JAKA App Real Robot Trajectory Execution.png" alt="JAKA App Real Robot Trajectory Execution">
  <figcaption>
    <p align="center"><strong>图 4-24：JAKA 应用程序真实机器人轨迹执行</strong></p>
  </figcaption>   
</figure>

### 运行 `moveit_test` 进行基本运动测试
`moveit_test` 可执行文件为用户提供了一种简单的方式，测试 MoveIt 2 在 JAKA 机器人上的运动规划和执行。它作为新用户的初步验证步骤，展示 JAKA ROS 2 包如何与 JAKA 机器人交互和控制。此测试可以在真实机器人、RViz 仿真模式或 Gazebo 仿真环境下进行。

##### `moveit_test` 的目的
- 为用户提供一个简单的起点，用于验证 MoveIt 2 的运动控制。
- 帮助测试 JAKA ROS 2 包与 JAKA 机器人之间的通信。
- 允许用户使用预定义的关节空间运动序列验证机器人的运动规划。

#### 使用真实机器人运行 moveit_test
要使用真实机器人进行测试，请确保在执行 `moveit_test` 之前已启动 MoveIt 服务器。  
在不同的终端中运行以下命令：

```bash
ros2 launch jaka_planner moveit_server.launch.py ip:=<robot_ip> model:=<robot_model>
ros2 launch jaka_<robot_model>_moveit_config demo.launch.py
ros2 run jaka_planner moveit_test --ros-args -p model:=<robot_model>
```

> **注意：**  
> - 将 `<robot_ip>` 替换为机器人实际的 IP 地址。
> - 将 `<robot_model>` 替换为相应的 JAKA 机器人型号。  
> - 对于 `moveit_test`，代码中默认的机器人型号设置为 `zu3`。如果使用不同型号，请在启动命令中指定它（`-p model:=<robot_model>`）。


<figure id="figure-4-25">
  <img src="images/Figure 4-25: MoveIt Test Executable.png" alt="MoveIt Test Executable">
  <figcaption>
    <p align="center"><strong>图 4-25：MoveIt 测试可执行文件命令输出</strong></p>
  </figcaption>   
</figure>

<figure id="figure-4-26">
  <img src="images/Figure 4-26: MoveIt Test Real Robot Execution.png" alt="MoveIt Test Real Robot Execution">
  <figcaption>
    <p align="center"><strong>图 4-26：MoveIt 测试真实机器人执行</strong></p>
  </figcaption>   
</figure>

#### 在 RViz 仿真模式下运行 moveit_test
要在 RViz 模拟模式下进行测试（无需实体机器人或三维仿真软件如 Gazebo），请省略 MoveIt 服务器启动，并将 `use_rviz_sim` 参数设置为 `true` 启动 RViz 演示。  
在不同的终端中运行以下命令：

```bash
ros2 launch jaka_<robot_model>_moveit_config demo.launch.py use_rviz_sim:=true
ros2 run jaka_planner moveit_test --ros-args -p model:=<robot_model>
```  

> **注意：** 将 `<robot_model>` 替换为相应的 JAKA 机器人型号。  

<figure id="figure-4-27">
  <img src="images/Figure 4-27: MoveIt Test RViz Simulation Mode Execution.png" alt="MoveIt Test RViz Simulation Mode Execution">
  <figcaption>
    <p align="center"><strong>图 4-27：MoveIt 测试 RViz 仿真模式执行</strong></p>
  </figcaption>   
</figure>  

#### 在 Gazebo 模拟下运行 moveit_test  
要在没有实际 JAKA 机器人的情况下使用 Gazebo 模拟执行测试，请省略 MoveIt 服务器启动，并将启动 Gazebo 或带有 RViz 的 Gazebo 演示。

**(1) 在独立的 Gazebo 模拟中运行 moveit_test：**  
在不同的终端中运行以下命令：

```bash
ros2 launch jaka_<robot_model>_moveit_config gazebo.launch.py 
ros2 run jaka_planner moveit_test --ros-args -p model:=<robot_model>
```  

> **注意：** 将 `<robot_model>` 替换为相应的 JAKA 机器人型号。  

<figure id="figure-4-28">
  <img src="images/Figure 4-28: MoveIt Test Independent Gazebo Execution.png" alt="MoveIt Test Independent Gazebo Execution" width="1200">
  <figcaption>
    <p align="center"><strong>Figure 4-28: MoveIt 测试独立的 Gazebo 仿真执行</strong></p>
  </figcaption>   
</figure>

**(2) 在带有 RViz 的 Gazebo 演示模拟中运行 moveit_test：**  
在不同的终端中运行以下命令：

```bash
ros2 launch jaka_<robot_model>_moveit_config demo_gazebo.launch.py 
ros2 run jaka_planner moveit_test --ros-args -p model:=<robot_model>
```  

> **注意：** 将 `<robot_model>` 替换为相应的 JAKA 机器人型号。  

<figure id="figure-4-29">
  <img src="images/Figure 4-29: MoveIt Test RViz for Demo Gazebo Execution.png" alt="MoveIt Test RViz for Demo Gazebo Execution">
  <figcaption>
    <p align="center"><strong>Figure 4-29: MoveIt 测试演示 RViz 为 Gazebo 仿真执行</strong></p>
  </figcaption>   
</figure>

<figure id="figure-4-30">
  <img src="images/Figure 4-30: MoveIt Test Demo Gazebo with RViz Execution.png" alt="MoveIt Test Demo Gazebo with RViz Execution" width="1200">
  <figcaption>
    <p align="center"><strong>Figure 4-30: MoveIt 测试带有 RViz 的 Gazebo 演示模拟执行</strong></p>
  </figcaption>   
</figure>