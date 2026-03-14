# QMI8658A IMU 驱动说明文档

## 1. 简介
本项目基于 ST 公司编码风格与分层架构，实现了 QMI8658A 六轴传感器（加速度计 + 陀螺仪）的驱动程序。
驱动支持初始化自检、运行时错误自动恢复、动态低通滤波等高级功能，适用于高可靠性姿态解算场景。

## 2. 架构说明

驱动采用经典的三层架构，实现硬件与业务逻辑的解耦：

### 2.1 应用层 (Application Layer)
*   **文件**: `imu.c`, `imu.h`
*   **职责**: 
    *   提供面向业务的接口（如 `IMU_Init`, `IMU_GetData`）。
    *   实现复杂逻辑：自动重连、动态滤波算法、静止检测状态机。
    *   数据单位统一为物理量（g, dps, °C）。

### 2.2 驱动层 (Driver Layer)
*   **文件**: `QMI8658A.c`, `QMI8658A.h`
*   **职责**:
    *   实现 QMI8658A 寄存器操作（配置、读取）。
    *   数据转换（LSB 转物理量）。
    *   不依赖具体硬件平台，仅通过 `stmdev_ctx_t` 接口与底层交互。

### 2.3 平台层 (Platform Layer)
*   **文件**: `platfrom_qmi8658a.c`, `platfrom_qmi8658a.h`
*   **职责**:
    *   适配 STM32 HAL 库（I2C 读写、延时、时间戳）。
    *   隔离硬件差异，若移植到其他 MCU 仅需修改此层。

---

## 3. 关键功能特性

### 3.1 初始化自检
在 `IMU_Init` 中，驱动会自动读取 5 次数据进行校验。若数据全为 0 或通信失败，会自动触发软复位并返回错误，确保启动时传感器状态正常。

### 3.2 运行时自动恢复
在 `IMU_GetData` 中包含错误计数器：
*   **通信失败**: 连续 10 次 I2C 读取失败，触发重置。
*   **数据异常**: 连续 20 次数据全为 0，触发重置。
*   **机制**: 重置后，下一次调用 `IMU_GetData` 会自动重新执行初始化流程。

### 3.3 动态低通滤波 (Dynamic LPF)
为平衡**静态稳定性**与**动态响应速度**，驱动实现了动态滤波算法：
*   **静止时**: Alpha = 0.05 (强滤波)，消除零漂，数据极稳。
*   **运动时**: Alpha = 0.60 (弱滤波)，响应灵敏，无延迟。
*   **过渡**: 根据陀螺仪模长在 2.0~20.0 dps 之间线性插值。

---

## 4. 接口文档

### 4.1 初始化
```c
/**
 * @brief  IMU 模块初始化
 * @retval IMU_OK: 成功
 *         IMU_ERROR: 失败 (需检查硬件连接)
 */
imu_status_code_t IMU_Init(void);
```

### 4.2 获取数据
```c
/**
 * @brief  获取经过滤波和校准的 IMU 数据
 * @param  data: 输出结构体 (acc, gyro, temp, timestamp)
 * @retval IMU_OK: 成功
 *         IMU_ERROR_NO_DATA: 失败 (可稍后重试，驱动会自动恢复)
 */
imu_status_code_t IMU_GetData(imu_data_t *data);
```

### 4.3 获取状态
```c
/**
 * @brief  获取静止检测状态
 * @param  status: 输出结构体 (no_motion_detected)
 */
imu_status_code_t IMU_GetStatus(imu_status_t *status);
```

---

## 5. 移植指南
若需移植到其他平台（如 ESP32, GD32）：
1.  **保留**: `imu.c`, `imu.h`, `QMI8658A.c`, `QMI8658A.h`。
2.  **修改**: `platfrom_qmi8658a.c`，重新实现 `stm32_i2c_write`, `stm32_i2c_read`, `platform_get_tick` 即可。

## 6. 待办事项 (TODO)
- [ ] 添加 Flash 存储接口，保存校准参数 (Bias)。
- [ ] 启用 FIFO 模式以支持更高频采样。


