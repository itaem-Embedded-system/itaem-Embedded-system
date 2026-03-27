# 嵌入式项目代码规范与管理方案

本文档旨在为嵌入式开发项目提供一套统一的代码风格指南、架构设计原则及项目管理方案，以提高代码的可复用性、可维护性及团队协作效率。

## 1. 架构设计原则 (Architecture Design)

为实现代码跨平台复用（如更换 MCU 后业务逻辑无需修改），本项目采用 **分层架构** 与 **依赖倒置** 原则。

### 1.1 分层架构
工程严格划分为以下三层，层间通过接口交互，禁止跨层依赖：

1.  **应用层 (Application)**: 
    *   负责业务流程、任务循环、用户交互。
    *   **只依赖中间件层提供的服务接口**，**绝不**直接调用底层硬件句柄（如 HAL 库）。
    *   *语言建议*：根据业务复杂度选择，可使用 C/C++，甚至 MicroPython/Lua 等脚本语言。
2.  **中间件层 (Middleware)**: 
    *   包含可复用的功能模块（如协议栈、算法模型、数据解析）。
    *   **核心逻辑与硬件无关**。通过“平台适配层”或“接口注入”与具体硬件交互。
    *   *语言建议*：算法与模型类模块（如 PID, Kalman Filter, 神经网络）推荐使用 C++ (Template, OOP) 以提高抽象效率，或 Rust 以提高安全性。//rust在嵌入式的应用较少，慎重考虑。
3.  **驱动层 (Device / Driver)**: 
    *   **主控驱动**：处理寄存器、中断、总线时序（与硬件强相关）。
    *   **外设驱动**：传感器、屏幕等模块驱动。应设计为“纯逻辑”，通过传入的 IO 接口操作硬件。
    *   *语言建议*：推荐使用 **C 语言** 以确保最佳的编译器兼容性与执行效率。

### 1.2 接口抽象与依赖倒置
*   **接口先行**：编写驱动时，先定义“能力接口”（如 `read`, `write` 函数指针），再做平台绑定。
*   **依赖方向**：上层依赖抽象接口，不依赖具体实现。
*   **平台适配**：在 `Drivers/platform` 目录下实现具体平台的接口绑定，避免“屎山”堆积。

---

## 2. 代码规范 (Code Style)

本规范主要针对 C/C++ 语言，适用于各类嵌入式开发环境。
**说明**：本规范主要针对底层驱动和核心逻辑。对于中间件、工具链或应用层脚本，在符合整体架构前提下，可灵活选择适合的语言（如 C++, Rust, Python 等），不强制拘泥于本 C 语言规范。

### 2.1 命名规范
*   **语言统一**：整个项目统一使用英语（推荐）或统一拼音，严禁混用。
*   **文件命名**：使用小写字母，单词间用下划线分隔（Snake Case）。
    *   例如：`adc_driver.c`, `motor_control.h`
*   **变量与函数**：
    *   **函数名**：`Module_Action` (Pascal Case) 或 `module_action` (Snake Case)，保持统一。
    *   **变量名**：清晰描述用途，拒绝无意义单字母（`i`, `j` 除外）。
    *   **全局/静态变量**：全局加 `g_`，静态加 `s_`，指针加 `p_`。
*   **类型定义**：
    *   **必须**使用 `stdint.h` 定义的定宽类型：`uint8_t`, `int16_t`, `uint32_t`, `float`。
    *   多文件访问的变量，建议使用 `struct` 或 `enum` 封装。

### 2.2 函数与结构（强制规则）
*   **变量定义前置（C89 风格）**：
    *   函数内所有局部变量定义**必须**放置在函数体最开头。
    *   **禁止**在代码块中间随意定义变量。
    ```c
    void Motor_Start(void)
    {
        /* 变量定义区 */
        uint8_t i;
        uint32_t status;
        
        /* 代码逻辑区 */
        status = 0;
        for (i = 0; i < 10; i++)
        {
            // ...
        }
    } // end of Motor_Start
    ```
*   **函数后注释**：
    *   每个函数结束的大括号 `}` 后，**必须**添加注释说明。
    *   格式：`} // end of FunctionName`
    ```c
    void Led_Init(void)
    {
        // ...
    } // end of Led_Init
    ```
*   **关键变量保护**：
    *   中断与多任务共享变量**必须**声明为 `volatile`。
    ```c
    volatile uint32_t g_TickCount; // 在 SysTick 中断中修改
    ```
*   **内存管理**：
    *   **慎用** `malloc`/`free` 动态内存分配，优先使用静态数组或内存池。

### 2.3 注释规范
*   **函数头注释**：必须包含“概要 - 参数 - 返回值”说明。
    ```c
    /**
     * @brief  发送数据包
     * @param  data 指向数据缓冲区的指针
     * @param  len  数据长度
     * @return 0: 成功, -1: 失败
     */
    int Uart_Send(uint8_t* data, uint16_t len);
    ```
*   **文件头注释**：
    ```c
    /**
     * @file    adc_driver.c
     * @brief   ADC Driver Implementation
     * @author  Team Embedded
     * @date    2023-10-01
     */
    ```
*   **代码段注释**：说明“为什么”这么做，而不是“做了什么”。
    ```c
    // 错误示例：将计数器加一
    // count++;
    
    // 正确示例：由于传感器有 10ms 启动延迟，此处需额外等待
    HAL_Delay(10);
    ```

---

## 3. 工程结构 (Project Structure)

建议采用以下目录结构，以支撑分层架构：

```
Project_Root/
├── Core/                   # CubeMX生成代码 (启动, 时钟, 中断, HAL库初始化)
├── Drivers/
│   ├── devices/            # 外设设备驱动逻辑 (OLED, Sensor) - 无硬件依赖纯逻辑
│   └── platform/           # 平台适配层 (实现具体MCU的接口)
│       └── stm32/          # STM32平台的适配实现 (如 gpio_stm32.c)
├── Middleware/             # 中间件 (MQTT, GPS解析, PID算法, 日志) - 跨项目复用
├── App/                    # 应用层 (main.c, task.c, 业务逻辑)
├── inc/                    # 全局头文件
│   └── interfaces.h        # 定义全项目的抽象接口 (函数指针结构体)
├── Doc/                    # 项目文档 (Readme, 原理图, 接口说明)
└── Output/                 # 编译输出
```

---

## 4. 可复用驱动开发指南 (Reusable Driver Guide)

**核心理念**：本节提供一种实现“代码解耦”与“跨平台复用”的**设计思路**，而非强制模板。
在实际开发中，请根据项目规模和团队习惯灵活调整。对于简单项目，可适当简化抽象层级；对于复杂系统，建议严格遵循以降低维护成本。

### 4.1 定义能力接口 (Interfaces)
在 `inc/interfaces.h` 中定义抽象接口结构体，不包含具体实现。

```c
// 示例：OLED 显示接口
typedef struct {
    void (*clear)(void);
    void (*show_string)(uint8_t x, uint8_t y, const char* str);
    void (*show_num)(uint8_t x, uint8_t y, uint32_t num, uint8_t len);
} oled_interface_t;
```

### 4.2 编写去耦驱动 (Drivers/devices)
编写驱动逻辑时，不调用 `HAL_GPIO_WritePin`，而是调用传入的底层接口函数。

### 4.3 平台适配实现 (Drivers/platform)
创建适配文件（如 `device_port_stm32.c`），实例化接口对象，并注入 HAL 库函数。

```c
// 1. 实现具体平台的函数
static void stm32_oled_clear(void) {
    OLED_Clear(); // 调用原始驱动或HAL库
}

// 2. 组装接口对象
const oled_interface_t stm32_oled_if = {
    .clear = stm32_oled_clear,
    .show_string = stm32_oled_show_string,
    // ...
};

// 3. 构造器：将接口注入到中间件或设备对象中
mqtt_device_t* mqtt_device_create_stm32(void) {
    return mqtt_device_create(&stm32_oled_if, ...);
}
```

### 4.4 应用层调用 (App)
业务代码仅操作抽象对象，不感知底层硬件。

```c
void App_Task(void) {
    // 初始化设备 (获取接口对象)
    mqtt_device_t* dev = mqtt_device_create_stm32();
    
    // 调用接口 (不直接调用 HAL 库)
    dev->oled_if.show_string(0, 0, "System Init");
}
```

---

## 5. 项目管理 (Project Management)

### 5.1 文档管理
*   **Readme.md**: 必须包含项目简介、结构说明、使用方法、待优化项。
*   **硬件资料**: 嵌入式项目应包含硬件配置、原理图、PCB (或链接)。
*   **模块文档**: 这里的每个复用模块（中间件/驱动）应附带接口清单和使用说明。

### 5.2 版本控制 (Git)
*   遵循 **Git Flow** (master/develop/feature)。
*   提交信息遵循 **Conventional Commits** (`feat`, `fix`, `docs` 等)。

### 5.3 调试与质量
*   **日志**: 多写日志显示代码，善用错误码辅助定位。
*   **代码审查**: 重点检查逻辑死循环、资源泄漏、规范符合度。
