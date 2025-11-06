# arm_voice_insterface

Lightweight ROS2 Python package that uses Vosk for local STT and publishes normalized arm commands to `/arm_command`.

Installation
1. Install Python dependencies (recommend virtualenv):

```bash
pip install -U pip
pip install vosk sounddevice
```

2. Download a Vosk model (small Chinese model recommended for low-resource):
下载并解压 Vosk 模型（例如小型中文模型），放到 ~/models/vosk-model-small-cn 或指定 VOSK_MODEL_PATH 环境变量。

```bash
mkdir -p ~/models
cd ~/models
# choose a model from https://alphacephei.com/vosk/models
# example: vosk-model-small-cn-0.22
wget <model_url> -O model.tgz
tar xzf model.tgz
mv vosk-model-* vosk-model-small-cn
```

3. Build ROS2 packages and source workspace:

```bash
cd ~/lododo-arm
colcon build --packages-select arm_voice_insterface --symlink-install
source install/setup.bash
```

Run

```bash
# start voice node
ros2 run arm_voice_insterface arm_voice_node

# or directly (after source)
python3 -m arm_voice_insterface.stt_node

# publish test command
ros2 topic pub /arm_command std_msgs/String "data: 'scan_front'"

# watch results
ros2 topic echo /arm_command_result
```

Notes
- For reliable behavior in production, use a wake-word or VAD to avoid false triggers.
- For better Chinese recognition accuracy consider using a larger Whisper-based model if CPU permits.

报错显示缺少 PortAudio（本地 native 库），这是 sounddevice 的依赖。解决步骤如下，我可以帮你在此环境里执行（需要系统包安装权限）：

修复步骤（按顺序）

1安装系统依赖（PortAudio）：

sudo apt update
sudo apt install -y libportaudio2 portaudio19-dev libsndfile1
2重新安装或升级 Python 包（为确保 sounddevice 能链接 PortAudio）：

# 推荐用用户级安装或虚拟环境；这里示例用 
pip install --user --upgrade pip
pip install --user --upgrade sounddevice
3验证（在同一 shell 中 source workspace 后运行）：

source install/setup.bashpython3 -c "import sounddevice, vosk; print('ok', sounddevice.__file__, vosk.__file__)"
如果导入成功，再次运行 ROS2 节点：

source install/setup.bashros2 run arm_voice_insterface arm_voice_node
我可以现在替你执行第1-3 步（在仓库主机上运行 apt + pip 并验证），如果你同意就回复“请执行 apt 安装并重装 sounddevice”，我会立刻运行并回报结果