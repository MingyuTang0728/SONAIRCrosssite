# LPMS-B2 直连：OpenZen + 自动重连 + 断线自动判废

## 为什么不再经过 FusionHub

FusionHub 是 LP-Research 的产品，但它是一个融合中间件（用于 VR 光学跟踪、车辆定位），
不是 LPMS-B2 的驱动。LPMS-B2 的官方工具是 **LPMS-Control**（包含在 OpenMAT 中）和 **OpenZen** SDK。
直接用 OpenZen 读传感器有三个好处：
- 少一层软件，少一个可能断线的环节；
- 拿到**传感器自己的时间戳**。FusionHub 转发的时间戳不走，以前只能按到达时间打时间戳，
  会带上蓝牙一批一批送达造成的抖动；
- 断线时 OpenZen 会明确报告，平台可以自己重连。

## 一次性安装（约 2 分钟）

在 PyCharm 里运行 **`0 - Install OpenZen + LPMS-Control`**（或在命令行执行
`python install_openzen.py --lpms-control`）。它会：
1. 下载 LP-Research 官方的 OpenZen（Python 3.11 版），以及 python.org 官方的嵌入式 Python 3.11。
   两者放在 `%LOCALAPPDATA%\SONAIR\openzen`（这台电脑上所有项目副本共用），**不改动你现有的 Python 3.12，也不改系统**；
2. 启动一次 OpenZen，确认能加载；
3. 下载 LPMS-Control（OpenMAT 1.3.5）并打开它的安装程序，按提示安装即可。

如果提示 `DLL load failed`，说明缺少 VC++ 运行库，安装
https://aka.ms/vs/17/release/vc_redist.x64.exe 后再运行一次。

## 第一步：先用 LPMS-Control 判断断线出在哪一层

1. 关掉 FusionHub；
2. 打开 LPMS-Control，连接 B2，输出设为 **100 Hz**，只勾选陀螺仪、加速度、四元数；
3. 让它持续连接 **30 分钟**：
   - **也断** → 问题在蓝牙或传感器本身：关闭 Windows 蓝牙省电，把蓝牙适配器放到远离 USB 3 和相机线的位置，
     给传感器充满电；
   - **不断** → 问题在 FusionHub 那一层，改用下面的直连方式即可。
4. 测完关闭 LPMS-Control：同一时间只能有一个程序占用传感器。

## 在平台里使用

Sensors 页 → 连接方式选 **"LPMS sensor directly over Bluetooth (OpenZen) — recommended"**：
1. 按 **Find it for me**：用约 12 秒搜索蓝牙上的 LPMS 传感器，并把地址自动填好；
2. 按 **Connect**；
3. 状态栏每 2 秒更新一次，例如：
   > Streaming from LPMSB2-4B3141 at 100 a second · battery 87% · no dropouts ·
   > timed by the sensor's own clock.

断线后**不需要任何操作**：
- 蓝牙掉线时，OpenZen 会报告断开，平台自动重连；
- 传感器"静默"不发数据（没有报断开）超过 2 秒，平台也会判定为断线并重连；
- 读传感器的子进程如果崩溃，平台会自动重启它；
- 状态栏会显示最近 10 分钟的掉线次数、最长中断时长，以及无线传输中丢失的数据比例
  （根据传感器自己的帧计数器计算）。

## 断线自动判废

- 每条录制数据都会记下当时 IMU 读数的"年龄"。传感器掉线时，文件里会反复写入最后一个读数，
  看起来像传感器完全静止；有了"年龄"，读回检查就能识别出来。
- **IMU 中断超过 0.25 秒**的运行会被判为不合格。批量实验中，这样的运行**不会被标记为完成**，
  再次运行该 session 时只会重录它。
- 需要 IMU 的任务在开始录制前会先确认 IMU 在线；如果正在掉线，会等它重连（最多 30 秒），
  而不是录一段注定要判废的数据。

## 技术说明

- OpenZen 在一个独立进程里运行（`openzen_bridge.py`），使用 `%LOCALAPPDATA%\SONAIR\openzen` 中的 Python 3.11（旧位置 `vendor/openzen` 也仍然可用）。
  这和机器人数据的读取方式一样：蓝牙驱动即使卡住，也不会拖慢 agent。
- 时间戳：取过去 10 秒内"到达时间 − 传感器时间"的最小值（也就是延迟最小的那次传送），
  用它把传感器时钟映射到电脑时钟上。这样能跟上两个时钟之间的缓慢漂移，并且映射后的时间
  不会晚于数据实际到达的时间。如果传感器时间戳停止前进，会自动改回按到达时间，并在状态栏说明。
- 单位：OpenZen 输出的是 g 和 度/秒，平台会换算成 m/s² 和 rad/s。
- 测试：`python tests/test_openzen_link.py` 用一个与官方接口同名的模拟 OpenZen
  （`tests/fake_openzen/`）验证：流式读取、时间戳、蓝牙掉线、静默中断、进程崩溃、丢帧统计、
  未安装时的提示、搜索传感器，以及断线运行被自动判废。
  **真实的 B2 只能在你的电脑上验证。**
