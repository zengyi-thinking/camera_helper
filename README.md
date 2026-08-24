# camera_helper

`flight_control` 的副摄视觉板工程。CM7_0持续采集MT9V03X灰度图，识别V型车灯与信标灯，并通过UART6向飞控返回V→B向量；CM7_1保留最小启动模板。

该工程用于补充主摄视野，不独立控制无人机或小车。主摄任务锁定和最终A4发布仍由飞控主板负责。

## 硬件与接线

- MCU：Infineon CYT4BB7。
- 摄像头：MT9V03X，当前按170度对角视场配置。
- 显示：IPS200 SPI。
- 主板通信：UART6，230400。
- 副摄TX：`P03_1`，连接主板RX。
- 副摄RX：`P03_0`，连接主板TX。

主板和副摄的TX/RX必须交叉连接，并共地。

## 工程结构

```text
code/application/camera_helper/  图像任务、显示、请求解析和响应发送
code/algorithm/vision/           V灯、信标、活动检测和镜头几何算法
code/hardware/                   视觉公共配置
user/                            双核入口
iar/                             IAR工程、构建和烧录脚本
libraries/                       本工程实际使用的SDK与逐飞外设库
```

副摄与主摄保留两份视觉算法源码，以便分别构建。修改 `code/algorithm/vision/` 后必须同步检查 `flight_control` 中的同名文件；公开发布前应确保两边只有明确记录的差异。

## 视觉流程

每个摄像头帧按以下顺序处理：

1. 运行 `algo_vision_target_pipeline_update()`。
2. 使用严格检测、亮区回退、局部恢复和短时保持识别V灯。
3. 检测最多三个信标候选，并应用V灯冲突排除和活动证据。
4. 对V灯与信标点分别做170度等距鱼眼几何校正。
5. 计算副摄自身坐标中的V→B向量，再执行90度安装映射。
6. 更新IPS200覆盖层和B2快照。

屏幕标记：绿色为V灯，红色为信标，黄色为V→B连线。

## 坐标映射

副摄自身图像坐标为X向右、Y向下：

```c
raw_dx = beacon_x - v_x;
raw_dy = beacon_y - v_y;
```

当前副摄相对主摄按逆时针90度安装：

```c
mapped_dx = -raw_dy;
mapped_dy =  raw_dx;
```

镜头校正在副摄自身坐标中完成，飞控主板不得再次校正 `mapped_dx/mapped_dy`。

## UART6协议

主板请求B1，固定5字节：

```text
0xB1 | seq | mode | XOR(0..2) | 0x55
```

副摄响应B2，固定27字节：

```text
0xB2 | seq | flags
| V_x V_y | beacon_x beacon_y
| raw_dx raw_dy | mapped_dx mapped_dy
| V_conf | beacon_conf | frame_id
| XOR(0..24) | 0x55
```

除单字节字段外均为大端序；坐标和差值为有符号 `int16`，置信度和帧号为无符号 `uint16`。

`flags`：

| 位 | 名称 | 含义 |
|---:|---|---|
| 0 | `HAS_V` | V灯有效 |
| 1 | `HAS_B` | 信标有效 |
| 2 | `MOVE_VALID` | V灯与信标同时有效 |
| 3 | `FRESH` | 快照未超过副摄新鲜窗口 |
| 4 | `MAPPED_VALID` | 90度映射向量有效 |

副摄收到合法B1后立即回传当前快照，同时保留低频心跳。飞控默认每20ms请求一次，220ms未收到有效响应则认为副摄链路失效。

## 与飞控任务的边界

- 副摄只返回当前确认的单个V灯和单个信标结果。
- 主摄任务队列、B1锁定、主副摄选择和A4发布由 `flight_control` 管理。
- 定时停车与Yaw任务由飞控CM7_1管理，副摄不单独计时。
- B1/B2、共享内存或A4标志发生变化时，应使用同一发布版本的三个工程一起烧录。

## 构建

工程已使用仓库内 `libraries/`，不再依赖相邻的 `E8_camera` 目录。

需要Windows和IAR Embedded Workbench for Arm 9.2系列。如果IAR不在 `PATH` 中：

```bat
set IAR_EW_ROOT=C:\path\to\Embedded Workbench 9.2
```

构建两个核心：

```bat
iar\build_all_cores.bat --no-pause
```

构建并烧录：

```bat
iar\download_all_cores.bat --no-pause
```

全新克隆第一次烧录前，需在IAR中分别打开 `iar/project_config/cyt4bb7_cm_7_0.eww` 和 `cyt4bb7_cm_7_1.eww`，选择正确调试器并让IAR生成本机C-SPY设置；这些个人设置不会提交到仓库。

## 验证

1. 遮挡/移开V灯和信标，确认屏幕标记与B2 flags一致。
2. 连续发送B1，确认B2序号跟随且帧长度、校验稳定。
3. 将信标分别移到V灯右、下、左、上，核对原始和90度映射向量。
4. 在主摄短时失去信标时，确认飞控能快速使用副摄结果；主摄恢复后不得切换到错误信标。

## AI协作：飞跃雷区 Skill

仓库内置 [`feiyue-leiqu`](.agents/skills/feiyue-leiqu/SKILL.md)，用于主副摄视觉、飞控、小车和通讯的协同设计。在本仓库目录启动 Codex 后会自动发现，也可以显式输入：

```text
$feiyue-leiqu 提升远距信标识别，同时约束误检和主摄接管风险
```

三工程同处 `Motherboard_Demo` 时，共享规范源位于 `../.agents/skills/feiyue-leiqu`；本仓库副本用于独立克隆。共同视觉算法或 B1/B2 语义变化时，应同步 skill 并配套验证飞控端。

## 许可证

本项目自研代码以 GNU General Public License v3.0 or later 发布，完整条款见 [`LICENSE`](LICENSE)。`libraries/` 中的第三方组件继续适用其各自的版权和许可证声明，详见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。
