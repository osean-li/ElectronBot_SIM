# electronBot 小智 AI 机器人 — 快速上手与源码编译指南

> 目标：新手上手，既能快速用起来，也能自己改代码编译做功能验证  
> 环境：Windows 10/11，ESP-IDF v5.5.4，ESP32-S3，electronBot  
> 本文档是你本地的操作手册。遇到以下问题时看对应的外部参考：

---

## 📚 外部参考资料（什么时候看）

| 参考链接 | 什么时候去看 |
|----------|------------|
| [electronbot.tech](https://electronbot.tech/) | ① 下载最新预编译固件（不用编译）；② 在线一键烧录；③ 查看组装教程、BOM 物料清单、FAQ 常见问题 |
| [飞书百科 - 小智 AI 聊天机器人百科全书](https://my.feishu.cn/wiki/F5krwD16viZoF0kKkvDcrZNYnhb) | ① 面包板手工 DIY 接线教程；② 各个开发板的详细硬件说明；③ 社区维护的各类教程合集（建议收藏） |
| [xiaozhi.me 控制台](https://xiaozhi.me) | 账号注册、设备绑定、大模型配置、音色切换 |
| [GitHub - 78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) | 查看最新源码更新、提 Issue、看其他开发者的问题讨论 |
| [B站视频教程](https://www.bilibili.com/video/BV1XnmFYLEJN/) | 手工打造 AI 女友新手入门视频，直观了解完整流程 |

---

## 两种方式，按需选择

| | 路径 A：快速使用 | 路径 B：源码编译 |
|---|---|---|
| **适合场景** | 刚拿到机器人，想立刻体验 | 想改代码、调试、做功能验证 |
| **需要什么** | 一根 USB 线 + Chrome 浏览器 | ESP-IDF 开发环境 + USB 线 |
| **耗时** | 5 分钟 | 首次 30~60 分钟，后续 5~10 分钟 |
| **能改代码吗** | 不能 | 能，改完编译烧录即可 |
| **跳到哪** | → [路径 A](#路径-a快速使用下载预编译固件烧录) | → [路径 B](#路径-b源码编译改代码--编译--烧录) |

> **推荐**：先走路径 A 验证硬件正常，再用路径 B 改代码做开发。

---

# 路径 A：快速使用（下载预编译固件烧录）

## A.1 下载固件

预编译固件在 [electronbot.tech 下载页](https://electronbot.tech/docs/downloads) 获取。

> 💡 这个网站还提供在线烧录、组装教程、PCB 下单、BOM 清单、焊接指南，需要硬件相关操作时优先看它。

卖家说的「用最新的 release」，就是下载最新版本 `v2.2.6-2`。

也可直接下载：

```
https://electronbot.tech/files/electronbot2.2.6-2.bin
```

> 这个 bin 文件是合并固件，包含 bootloader + 分区表 + 固件 + 资源包，一个文件烧录即可。

## A.2 烧录到机器人

### 方法一：浏览器在线烧录（最简单）

1. 用 Chrome/Edge 打开 https://electronbot.tech/docs/downloads
2. 用 USB Type-C 线连接 electronBot 到电脑
3. 点击页面上的「开始在线烧录」
4. 浏览器弹出串口选择框 → 选择你的 COM 口
5. 等待完成

> ⚠️ **首次烧录**或更换固件版本时：**先按住 BOOT 键不放，再插 USB 上电**，烧录工具识别到芯片后再松手。

### 方法二：ESPTool 图形工具（离线可用）

项目自带烧录工具：`tools/flash_download_tool_3.9.5/flash_download_tool_3.9.5.exe`

1. 双击打开
2. ChipType 选 **ESP32-S3**
3. 点 OK 进入主界面
4. 左上角选 COM 端口，波特率填 `921600`
5. 下方表格第一行：
   - 勾选 ✅
   - 地址填 `0x0`
   - 点 `...` 选择下载的 `electronbot2.2.6-2.bin`
6. 勾选 DoNotChgBin
7. 点击 **START**
8. 等进度条走完，重启机器人

> 波特率如果报错，降到 `115200` 重试。

## A.3 首次开机配置

1. 上电后按 BOOT 键进入配网模式
2. 手机连接 WiFi 热点 `xiaozhi-xxxx`
3. 浏览器打开配网页，输入 WiFi 密码
4. 访问 [xiaozhi.me](https://xiaozhi.me) 注册账号
5. 输入屏幕上显示的 6 位激活码绑定设备
6. 说「你好小智」开始对话

---

# 路径 B：源码编译（改代码 → 编译 → 烧录）

## B.1 你的环境确认

| 项目 | 路径 |
|------|------|
| ESP-IDF 安装目录 | `D:\software\Espressif\frameworks\esp-idf-v5.5.4` |
| IDF Python 环境 | `D:\software\Espressif\python_env\idf5.5_py3.11_env` |
| 编译工具链 | `D:\software\Espressif\tools\` |
| 项目源码 | `D:\lht\rebot\xiaozhi\xiaozhi-esp32` |

## B.2 打开 ESP-IDF 命令行

**这是最关键的一步**，所有 idf.py 命令必须在这个终端里运行。

Windows 上有三种方式：

```
方式 1（推荐）：开始菜单 → ESP-IDF 5.5 PowerShell
方式 2：开始菜单 → ESP-IDF 5.5 CMD
方式 3：手动执行 D:\software\Espressif\frameworks\esp-idf-v5.5.4\export.ps1
```

> 打开后终端会显示类似 `ESP-IDF v5.5.4` 的欢迎信息，说明环境激活成功。

## B.3 首次编译（一次性配置）

在 IDF 终端中依次执行：

```powershell
# 1. 进入项目目录
cd D:\lht\rebot\xiaozhi\xiaozhi-esp32

# 2. 清理旧的构建残留（如果有）
idf.py fullclean

# 3. 设置目标芯片为 ESP32-S3
idf.py set-target esp32s3
# 这一步会生成 sdkconfig，并应用 sdkconfig.defaults 和 sdkconfig.defaults.esp32s3

# 4. 打开配置菜单，选择板型
idf.py menuconfig
```

在 `menuconfig` 界面中：

```
用方向键导航，回车进入，空格选中，ESC 返回，? 帮助

导航路径：
  Xiaozhi Assistant  --->
    Board Type  --->
      [*] electronBot          ← 选中这个！卖家说的「用 electron-bot/」就是这个

还可以顺便检查：
  Xiaozhi Assistant  --->
    Default Language  --->
      (X) Chinese              ← 确认是中文

保存退出：一路 ESC 回到顶层 → Save → OK → Exit
```

> ⚠️ **如果不选 electronBot**，固件会按默认板卡编译，舵机、显示屏驱动全是错的，刷进去机器人不工作。

## B.4 编译

```powershell
# 完整编译（首次 30~60 分钟，后续增量编译几分钟）
idf.py build
```

编译成功后，控制台输出关键产物路径：

```
build/xiaozhi.bin                   ← 应用固件
build/bootloader/bootloader.bin     ← bootloader
build/partition_table/partition-table.bin ← 分区表
build/ota_data_initial.bin          ← OTA 数据
build/generated_assets.bin          ← 资源包（表情/字体/音频）
```

## B.5 烧录到机器人

### 方式 1：idf.py 一键烧录（编译完直接用）

```powershell
# 用 USB 连接机器人，先确定 COM 口号
# 设备管理器 → 端口(COM和LPT) → 查看是 COM 几

# 替换 COM3 为你的实际端口
idf.py -p COM3 flash

# 烧录 + 打开串口监视器（看日志）
idf.py -p COM3 flash monitor
```

> 首次烧录按住 BOOT 再上电。Ctrl+] 退出串口监视器。

### 方式 2：合并单文件再烧录

```powershell
# 先合并成单个 bin
esptool.py --chip esp32s3 merge_bin `
  -o merged-flash.bin `
  --flash_mode dio `
  --flash_size 16MB `
  0x0 build/bootloader/bootloader.bin `
  0x8000 build/partition_table/partition-table.bin `
  0xd000 build/ota_data_initial.bin `
  0x20000 build/xiaozhi.bin `
  0x800000 build/generated_assets.bin

# 烧录合并文件
esptool.py --chip esp32s3 --port COM3 --baud 921600 write_flash -z 0x0 merged-flash.bin
```

## B.6 日常开发循环（改代码 → 验证）

以后每次改完代码：

```powershell
# 在 IDF 终端中
cd D:\lht\rebot\xiaozhi\xiaozhi-esp32

# 增量编译（快，只编译改过的文件）
idf.py build
# 或者一步到位：
idf.py -p COM3 build flash monitor
```

> 不需要每次都 fullclean，只有改了 Kconfig 配置才需要重新 menuconfig。

## B.7 烧录常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `A fatal error occurred: Could not open COM3` | 端口不对或设备没插好 | 设备管理器确认 COM 号 |
| `Timed out waiting for packet header` | 芯片未进入下载模式 | 按住 BOOT，再按一下 RST（或重新插拔 USB） |
| 波特率 921600 报错 | 个别 USB 转串口芯片不稳定 | 降到 115200 |
| 烧录完机器人没反应 | 没重启 | 拔掉 USB 重新插上 |
| menuconfig 找不到 electronBot | 没先 set-target | 先执行 `idf.py set-target esp32s3` |

---

# 源码结构速览（给改代码的你）

```
xiaozhi-esp32/
├── main/
│   ├── main.cc              ← 入口，一般不用改
│   ├── application.cc/.h    ← 核心逻辑：事件驱动 + 状态机
│   ├── mcp_server.cc/.h     ← MCP 设备控制服务
│   ├── ota.cc/.h            ← OTA 升级
│   ├── audio/               ← 音频处理（输入/输出/唤醒词）
│   ├── protocols/           ← 网络协议（WebSocket/MQTT）
│   ├── display/             ← 显示屏驱动（LCD/OLED/LVGL）
│   ├── led/                 ← LED 控制
│   └── boards/              ← ⭐ 硬件板卡定义
│       ├── common/           ← 公共功能（WiFi、按键、背光）
│       └── electron-bot/    ← electronBot 专属
│           ├── config.h            ← 引脚定义（改引脚看这里）
│           ├── electron_bot.cc     ← 板卡初始化
│           ├── electron_bot_controller.cc ← 舵机控制
│           ├── movements.cc/.h     ← 预置动作（改动作看这里）
│           ├── electron_emoji_display.cc  ← 表情显示
│           └── power_manager.h     ← 电量管理
├── partitions/v2/16m.csv   ← 分区表（V2，双 OTA）
├── sdkconfig.defaults.esp32s3 ← ESP32-S3 默认配置
└── tools/                  ← flash_download_tool 烧录工具
```

**改代码常用入口**：

| 想改什么 | 改哪个文件 |
|---------|-----------|
| 舵机动作/编排 | `main/boards/electron-bot/movements.cc` |
| 引脚映射 | `main/boards/electron-bot/config.h` |
| 唤醒词 | `main/audio/wake_words/` |
| 设备控制 MCP 命令 | `main/mcp_server.cc` |
| 表情/显示 | `main/boards/electron-bot/electron_emoji_display.cc` |
| WiFi 配网逻辑 | `main/boards/common/wifi_board.cc` |

---

# 协议与架构（了解即可）

## 语音交互流程

```
用户说话 → 唤醒词检测(ESP-SR) → 录音 → OPUS编码
    → WebSocket/MQTT → 服务器(ASR→LLM→TTS)
    → OPUS音频流返回 → 解码播放
```

## V2 分区表（16MB Flash）

```
地址       分区名     大小    用途
0x0000     (未分配)   36KB   二级 bootloader 实际用
0x9000     nvs        16KB   WiFi 配置存储
0xD000     otadata    8KB    OTA 状态
0xF000     phy_init   4KB    PHY 参数
0x20000    ota_0      ~4MB   ★ 当前运行固件
紧跟       ota_1      ~4MB   ★ OTA 备用固件（回滚用）
0x800000   assets     8MB    SPIFFS 资源包
```

**V1 和 V2 不兼容的原因**：V1 固件从 `0x10000` 开始，V2 从 `0x20000` 开始。从 V1 到 V2 必须全量 USB 烧录一次，之后就可以 OTA 升级了。

---

# 快捷命令速查

```powershell
# ===== 在 ESP-IDF 终端中执行 =====

# 配置
idf.py set-target esp32s3        # 设置芯片（只需一次）
idf.py menuconfig                # 图形化配置
idf.py fullclean                 # 清理所有构建产物

# 编译
idf.py build                     # 完整/增量编译

# 烧录（COM3 替换为你的端口）
idf.py -p COM3 flash             # 仅烧录
idf.py -p COM3 flash monitor     # 烧录 + 看日志
idf.py -p COM3 monitor           # 只看日志（Ctrl+] 退出）

# 合并固件为单文件
esptool.py --chip esp32s3 merge_bin -o merged.bin `
  --flash_mode dio --flash_size 16MB `
  0x0 build/bootloader/bootloader.bin `
  0x8000 build/partition_table/partition-table.bin `
  0xd000 build/ota_data_initial.bin `
  0x20000 build/xiaozhi.bin `
  0x800000 build/generated_assets.bin

# 命令行烧录单文件
esptool.py --chip esp32s3 --port COM3 --baud 921600 `
  write_flash -z --flash_mode dio --flash_size detect 0x0 merged.bin
```
