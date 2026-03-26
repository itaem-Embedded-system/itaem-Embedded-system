# MAXM10S 快速上手

当前工程最常用的只有 3 步。

## 1. 怎么接

- GNSS 模块串口默认按 38400 使用。
- 当前工程示例里，GNSS 接 USART1，
- 模块持续输出 NMEA 原始语句，STM32 负责接收、解析和筛选。

## 2. 怎么用

1. 初始化一次：`GNSS_maxm10s_init(&huart1);`
2. 简单业务优先调：`GNSS_maxm10s_update_and_get_location(&lat, &lon);`来快速获得经纬度
3. 需要更多字段时，再用 `GNSS_maxm10s_update();更新数据包` ，再调用各类 gett函数获取具体值。

当前最常用的 getter：

- `GNSS_maxm10s_get_location(&lat, &lon)`：拿经纬度
- `GNSS_maxm10s_get_altitude(&alt_m)`：拿海拔
- `GNSS_maxm10s_get_speed_kmh(&speed_kmh)`：拿速度
- `GNSS_maxm10s_get_heading(&heading_deg)`：拿航向
- `GNSS_maxm10s_get_local_time(&time)`：拿本地时间，默认 UTC + 8
- `GNSS_maxm10s_is_valid(&valid)`：看最近一次有效定位是否可用

便捷接口会先更新内部状态，再返回最近一次有效经纬度。

这些 getter 读取的是最近一次有效定位，可以重复读，不会把数据取走。

## 3. main.c 最短示例

```c
static double s_gnss_lat_deg;
static double s_gnss_lon_deg;
static char s_gnss_uart2_line[96];

GNSS_maxm10s_init(&huart1);

while (1)
{
    if (GNSS_maxm10s_update_and_get_location(&s_gnss_lat_deg, &s_gnss_lon_deg)) {
        int len = snprintf(s_gnss_uart2_line,
                           sizeof(s_gnss_uart2_line),
                           "lat=%.7f,lon=%.7f\r\n",
                           s_gnss_lat_deg,
                           s_gnss_lon_deg);
        if (len > 0) {
            HAL_UART_Transmit(&huart2,
                              (uint8_t *)s_gnss_uart2_line,
                              (uint16_t)strlen(s_gnss_uart2_line),
                              100);
        }
    }
}
```

## 4. 如果还想拿别的数据

单入口便捷接口：

- `GNSS_maxm10s_update_and_get_location(&lat, &lon)`

普通用法：

- `GNSS_maxm10s_update()`：刷新内部状态
- `GNSS_maxm10s_get_location()` 等 getter：从同一份快照里读取字段

常用数据获取：

- 经纬度：`GNSS_maxm10s_get_location`
- 海拔：`GNSS_maxm10s_get_altitude`
- 速度：`GNSS_maxm10s_get_speed_kmh`
- 航向：`GNSS_maxm10s_get_heading`
- 本地时间：`GNSS_maxm10s_get_local_time`

低频数据继续走整包接口：

- `GNSS_maxm10s_get_latest_fix(&fix)`

可以从 `fix` 里拿到：

- `sats`
- `hdop`
- `pdop`
- `fix_type`
- 原始 UTC 时间 `fix.utc`

状态和故障走：

- `GNSS_maxm10s_get_status(&status)`

## 5. 什么时候会返回 false

以下情况 getter 会返回 `false`：

- 还没有拿到任何有效定位
- 当前没有形成可读的最近有效快照
- 本地时间缺少年月日或时分秒

## 6. 当前有效定位规则

当前工程只有满足下面条件，才认为定位有效：

- 状态有效，RMC/GLL 里是 `A`
- 经纬度存在且数值正常
- 卫星数 `>= 4`
- `HDOP` 和 `PDOP` 大于 0 且小于 99.99
- `fix_type >= 2`

这表示的是“当前工程定义的可用门限”，不是模块协议里唯一的官方结论。

## 7. 当前保护机制

- 启动后给 1 分钟宽限
- 连续 3 秒收不到 NMEA，判定无数据超时
- 连续 1 分钟拿不到有效定位，判定搜星失败
- 失败后会做软恢复
- 最多自动恢复 5 次，再进入 FAULT

## 8. 一句话区分 3 个高级接口

- `GNSS_maxm10s_get_fix(&fix)`：消费式，来一帧取一帧
- `GNSS_maxm10s_get_latest_fix(&fix)`：非消费式，随时拿最近一次有效定位
- `GNSS_maxm10s_get_status(&status)`：看状态机和故障统计



