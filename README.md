# Practice_Program

RC 控制组培训练习工程集合。

## 目录约定

本仓库采用「一个子文件夹 = 一个独立工程」的结构，根目录只放仓库级文件：

```
Practice_Program/
├── .gitignore                       # 全局忽略规则，对任意子工程生效
├── README.md
└── 01_motor_control_freertos/       # 工程一：STM32F405 + FreeRTOS 电机控制
    ├── test9_9.ioc                  # CubeMX 配置源文件
    ├── Core/                        # CubeMX 生成：main.c、中断、MSP
    ├── App/                         # 应用层（FreeRTOS 任务与业务逻辑）
    ├── motor/  CAN/  PID/           # 电机、CAN 通信、PID 算法
    ├── Drivers/  Middlewares/       # HAL 库与 FreeRTOS 源码
    ├── MDK-ARM/  EWARM/             # Keil / IAR 工程文件
    └── CMakeLists.txt               # CMake 构建入口（CLion 使用）
```

## 在 CLion 中打开（重要）

**必须打开具体的工程子文件夹，不要打开仓库根目录。**

CLion 的 CMake 工程是 `CMakeLists.txt` + `CMakePresets.json` 所在的那一层，
也就是 `01_motor_control_freertos/`。如果在仓库根目录打开 CLion，根目录下没有
`CMakeLists.txt`，CLion 找不到任何 target，打开任何源码都会提示：

> 此文件不属于任何项目目标（This file does not belong to any project target）

此时源码索引、跳转、补全全部失效。正确做法：

1. `File` → `Open`（或启动界面的 `Open`）
2. 选择 **`01_motor_control_freertos`** 这一层目录，不要选它的上层
3. 确认 CMake 面板里出现 `test9_9` target

> 为什么不把根目录也做成 CMake 工程？因为 CubeMX 生成的
> `cmake/gcc-arm-none-eabi.cmake` 用 `${CMAKE_SOURCE_DIR}` 定位链接脚本
> `STM32F405xx_FLASH.ld`。若把子工程用 `add_subdirectory()` 挂到根工程下，
> `CMAKE_SOURCE_DIR` 会变成仓库根，链接脚本路径失效导致链接失败。
> 改动该文件会破坏 CubeMX 的可重新生成性，因此保持「每个工程独立打开」。

若曾在根目录打开过并留下了 `test9_9/.idea/`，该目录已被 `.gitignore` 忽略，
不影响仓库，可以随手删掉。

## 工程列表

| 目录 | 说明 | 构建方式 |
|---|---|---|
| `01_motor_control_freertos/` | STM32F405RGTx + FreeRTOS 电机控制，含 CAN 通信、串级 PID、UART 命令与遥测 | CMake + CLion / Keil MDK / IAR |

## 新增工程的步骤

1. 在仓库根目录新建子文件夹，建议按 `NN_工程名` 编号命名，例如 `02_can_bus_monitor/`。
2. 把 CubeMX / CLion 工程整体放入该文件夹（`.ioc`、`Core/`、`CMakeLists.txt` 等）。
3. 若使用 Keil 编译，在根目录 `.gitignore` 的「Keil 编译产物目录」处追加一行
   `**/MDK-ARM/<工程名>/`，避免把编译产物提交上来。
4. 在下方工程列表表格中补一行说明。

## 约定

- **不要**在子工程内部再建 `.git`，整个仓库共用根目录的一个仓库。
- 忽略规则统一写在根目录 `.gitignore`，子工程内不再单独维护。
- CMake 工程请使用 `${sourceDir}` 相对路径（参考 `CMakePresets.json`），
  这样整个文件夹可以随意移动位置而不需要改配置。
- 构建产物（`build/`、`MDK-ARM/<工程名>/`、`EWARM/settings/`）不入库。

## 工具链

- 交叉编译器：`arm-none-eabi-gcc`（GNU Tools for STM32）
- 构建：CMake ≥ 3.22 + Ninja，配置预设见各工程的 `CMakePresets.json`
