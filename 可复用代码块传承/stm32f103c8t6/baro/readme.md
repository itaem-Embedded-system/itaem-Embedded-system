
# SPL06 使用说明

这份文档是给“要把 SPL06 快速迁移到另一个 STM32 工程里的人”用的。

核心目标：

让别人把必要文件复制过去，按固定步骤调用几个函数，就能把 SPL06 跑起来。

## 1. 这个工程提供了什么
：

1. 通过 SPI 和 SPL06 通信。
2. 读取原始温度、原始气压。
3. 做补偿计算，得到温度、气压、海拔。
4. 默认启用动态滤波。
5. 通过 VOFA 回传 4 路数据。

也就是说，这个工程已经可以作为“可复制的 SPL06 模板”。

## 2. 迁移到新工程时，至少要复制哪些文件

如果你只是想把 SPL06 功能复制到另一个 STM32 工程，最少要迁移下面这些文件：

1. [app/baro.c](app/baro.c)
2. [app/baro.h](app/baro.h)
3. [device/spl06.c](device/spl06.c)
4. [device/spl06.h](device/spl06.h)
5. [platform/stm32ofspl06.c](platform/stm32ofspl06.c)
6. [platform/stm32ofspl06.h](platform/stm32ofspl06.h)

如果你也想直接复用当前串口波形回传，再额外参考：

1. [app/vofa.h](app/vofa.h)
2. 当前工程在 [Core/Src/main.c](Core/Src/main.c#L199) 和 [Core/Src/main.c](Core/Src/main.c#L220) 里的最小 VOFA 发送实现

如果你是整工程模板复制，则建议连下面这些一起带走：

1. [CMakeLists.txt](CMakeLists.txt#L46)
2. [docs/返回值.md](docs/返回值.md)
3. [docs/模板工程迁移清单.md](docs/模板工程迁移清单.md)

## 3. 新工程里必须满足的前提

迁移前，新工程至少要先具备这些基础条件：

1. 已经用 CubeMX 配好 SPI 外设。
2. 已经有一个可用的 `SPI_HandleTypeDef`。
3. 已经有 SPL06 的片选 GPIO。
4. 已经有 `HAL_Delay()` 和 `HAL_GetTick()`。
5. 如果要看波形，已经配好一个 UART。

## 4. 当前工程已经验证通过的关键配置

### 4.1 SPI 模式

当前工程已经验证通过并固定为：

1. `Mode0`
2. `CPOL = Low`
3. `CPHA = 1Edge`

对应代码在 [Core/Src/main.c](Core/Src/main.c#L103)。

### 4.2 接线

当前工程默认接线：

1. `SCK` -> `PA5`
2. `SDD/SDO` -> `PA6`
3. `SDI` -> `PA7`
4. `CSB` -> `PA10`
5. `VCC` -> `3.3V`
6. `GND` -> `GND`

说明：

1. `SDI` 接 MCU 的 `MOSI`
2. `SDD/SDO` 接 MCU 的 `MISO`
3. `CSB` 低电平有效

## 5. 主要调用哪些函数

如果你只关心“我到底该调哪几个函数”，看这一节就够了。

### 5.1 业务层最常用函数

对外最核心的是 [app/baro.h](app/baro.h) 里的 4 个函数：

1. `Baro_Init()`
2. `Baro_Run()`
3. `Baro_GetLatestData()`
4. `Baro_IsReady()`

你真正日常会调的，就是这 4 个。

### 5.2 平台层入口函数

平台层提供的入口是：

1. `STM32OF_GetSPL06Ctx()`

定义在 [platform/stm32ofspl06.h](platform/stm32ofspl06.h)。

一般情况下，你不用在业务层直接调用它，它主要是给 [app/baro.c](app/baro.c) 内部初始化时使用。

### 5.3 底层驱动核心函数

底层 SPL06 驱动提供的主要函数在 [device/spl06.h](device/spl06.h)：

1. `SPL06_Init()`
2. `SPL06_GetID()`
3. `SPL06_ReadRawTemp()`
4. `SPL06_ReadRawPress()`
5. `SPL06_Sleep()`
6. `SPL06_Wakeup()`

如果你只是复用当前模板，通常不需要在 main 里直接调用这些底层函数。

## 6. 新工程里最小调用流程

最小调用顺序如下：

1. 先初始化 GPIO
2. 再初始化 SPI
3. 再初始化 UART（如果要回传）
4. 锁定 SPI 为 Mode0
5. 调用 `Baro_Init()`
6. 主循环里周期调用 `Baro_Run()`
7. 用 `Baro_GetLatestData()` 取最新数据

当前工程在 [Core/Src/main.c](Core/Src/main.c#L99) 之后就是按这个流程组织的。

可以把它理解成下面这个最小模板：

```c
MX_GPIO_Init();
MX_SPI1_Init();
MX_USART2_UART_Init();

hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
HAL_SPI_Init(&hspi1);

Baro_Init();

while (1)
{
	Baro_Run();

	Baro_Data_t baroData;
	if (Baro_GetLatestData(&baroData)) {
		// 在这里使用 temperature / pressure / altitude
	}

	HAL_Delay(10);
}
```

## 7. 新工程必须同步改的地方

把文件复制过去还不够，下面这些地方也必须同步：

### 7.1 CMakeLists，只针对vscode环境

如果新工程使用 CMake，要把这 3 个源文件加进去：

1. [app/baro.c](app/baro.c)
2. [device/spl06.c](device/spl06.c)
3. [platform/stm32ofspl06.c](platform/stm32ofspl06.c)

当前工程写法见 [CMakeLists.txt](CMakeLists.txt#L46)。

### 7.2 头文件包含路径

要保证 include path 里包含：

1. `app/`
2. `device/`
3. `platform/`

### 7.3 平台适配文件

[platform/stm32ofspl06.c](platform/stm32ofspl06.c) 里这几件事和板子绑定：

1. 具体用哪个 SPI 句柄
2. CS 用哪个 GPIO 口和引脚
3. SPI 读写超时和重试次数

如果你换了 SPI 实例或者换了片选脚，这里必须改。

### 7.4 main 里的平台依赖函数

[app/baro.c](app/baro.c) 依赖这两个函数：

1. `App_DelayMs()`
2. `App_GetTickMs()`

当前实现放在 [Core/Src/main.c](Core/Src/main.c#L176)。
新工程里要保留这两个函数，或者提供等价实现。

## 8. 上电后你应该看到什么

如果工程正常，VOFA 会看到 4 个通道：

1. 温度
2. 气压
3. 海拔
4. tick 时间戳

具体含义见 [docs/返回值.md](docs/返回值.md)。

## 9. 你最常改的参数

### 9.1 改滤波参数

文件： [app/baro.c](app/baro.c#L18)

最常改的是：

1. `BARO_LPF_ALPHA_*`
2. `BARO_*_DELTA_*_TH`

经验：

1. 想更稳：减小低档和中档 alpha
2. 想更快：增大高档 alpha

### 9.2 改采样周期

文件： [app/baro.c](app/baro.c#L14)

参数：

1. `SENSOR_UPDATE_PERIOD_MS`

### 9.3 改启动瞬态处理

文件： [app/baro.c](app/baro.c#L16)

参数：

1. `SENSOR_WARMUP_SAMPLES`

如果你发现上电前几帧仍有尖峰，可以适当增大。

### 9.4 改复位后发布门控参数

文件： [app/baro.c](app/baro.c#L19)

这组参数控制“复位后什么时候允许对外发布数据”：

1. `SENSOR_PUBLISH_STABLE_SAMPLES`
2. `SENSOR_PUBLISH_PRESS_STABLE_TH`
3. `SENSOR_PUBLISH_ALT_STABLE_TH`

逻辑是：

1. 内部滤波照常快速运行。
2. 但在连续稳定样本达到阈值前，`Baro_GetLatestData()` 不会发布新数据。
3. 达到稳定条件后再开始正常发布。

这样可以屏蔽 MCU 复位后的尖峰污染帧，同时避免用硬限幅拖慢恢复速度。

参数经验：

1. 仍有复位尖峰穿透：增大 `SENSOR_PUBLISH_STABLE_SAMPLES`，或减小两个稳定阈值。
2. 觉得复位后“出数太慢”：减小 `SENSOR_PUBLISH_STABLE_SAMPLES`，或适当放宽稳定阈值。



如果你只是想把 SPL06 快速复制到另一个 STM32 工程：

1. 复制 `app/baro.*`、`device/spl06.*`、`platform/stm32ofspl06.*`
2. 把源文件加进 `CMakeLists.txt`
3. 改 `platform/stm32ofspl06.c` 里的 SPI 和 CS
4. 在 main 里锁定 SPI Mode0
5. 调 `Baro_Init()`、循环调 `Baro_Run()`、再用 `Baro_GetLatestData()` 取数

做到这 5 步，基本就能把这个芯片复制过去跑起来。
