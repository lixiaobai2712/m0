# CY-Z 陀螺仪协议解析

## 串口参数

- 接口：USART1
- 波特率：115200
- 数据位：8
- 校验位：无
- 停止位：1
- 流控：无

## 数据读取

CY-Z 使用固定 16 字节二进制帧输出陀螺仪角度和角速度数据。一帧内同时包含当前积分角度和滤波后的输出角速度。

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | Header | uint8[2] | 固定 `AA 55` |
| 2 | 2 | Seq | uint16 little-endian | 帧序号，每发送一帧加 1，溢出后回到 0 |
| 4 | 4 | AngleDeg | float32 little-endian | 陀螺仪积分角度，单位 deg |
| 8 | 4 | GyroDps | float32 little-endian | 陀螺仪角速度，单位 deg/s |
| 12 | 2 | CRC16 | uint16 little-endian | CRC-16/MODBUS |
| 14 | 2 | Tail | uint8[2] | 固定 `55 AA` |

帧结构：

```text
AA 55  Seq[2]  AngleDeg[4]  GyroDps[4]  CRC16[2]  55 AA
```

CRC 计算范围：

```text
Seq[2] + AngleDeg[4] + GyroDps[4]
```

CRC 不包含 `Header`、`CRC16` 和 `Tail`。

## CRC16 规则

- 算法：CRC-16/MODBUS
- 初值：`0xFFFF`
- 多项式：`0xA001`
- 输出：低字节在前，高字节在后

## Python 解析示例

```python
import struct


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


frame = bytes.fromhex("AA 55 01 00 00 00 80 3F 00 00 00 00 2A 07 55 AA")

if len(frame) == 16 and frame[0:2] == b"\xAA\x55" and frame[14:16] == b"\x55\xAA":
    payload = frame[2:12]
    crc_recv = struct.unpack_from("<H", frame, 12)[0]
    if crc16_modbus(payload) == crc_recv:
        seq, angle_deg, gyro_dps = struct.unpack_from("<Hff", frame, 2)
        print(seq, angle_deg, gyro_dps)
```

## 下行命令帧

上位机向 CY-Z 发送固定 8 字节命令帧，用于读取数据、角度清零和校准。

```text
A5 5A  Cmd[1]  Param[1]  Seq[1]  CRC16[2]  5A
```

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | Header | uint8[2] | 固定 `A5 5A` |
| 2 | 1 | Cmd | uint8 | 命令号 |
| 3 | 1 | Param | uint8 | 命令参数 |
| 4 | 1 | Seq | uint8 | 命令序号，响应帧原样返回 |
| 5 | 2 | CRC16 | uint16 little-endian | CRC-16/MODBUS |
| 7 | 1 | Tail | uint8 | 固定 `5A` |

CRC 计算范围：

```text
Cmd[1] + Param[1] + Seq[1]
```

## 命令定义

| Cmd | Param | 功能 | 静止要求 | 响应 |
|---:|---:|---|---|---|
| `0x01` | `0x01` | 角度清零，只清除当前积分角度 | 必须静止 | ACK 帧 |
| `0x01` | `0x02` | 零偏校准，重新估计软件零偏并清除积分角度 | 必须静止 | ACK 帧 |
| `0x04` | `0x00` | 读取陀螺仪角度和角速度 | 无 | 16 字节数据帧 |
| `0x05` | `1/2/3/6` | 开始比例校准，参数为旋转圈数 | 开始前必须静止 | ACK 帧 |
| `0x06` | `0x00` | 完成比例校准并保存因子 | 完成时必须静止 | ACK 帧 |
| `0x07` | `0x00` | 取消比例校准，不保存 | 无 | ACK 帧 |
| `0x08` | `0x00` | 读取当前比例因子 | 无 | 比例因子响应帧 |

## ACK 帧

CY-Z 收到有效命令并执行后，返回固定 8 字节 ACK 帧。

```text
A5 5B  Cmd[1]  Result[1]  Seq[1]  CRC16[2]  5B
```

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | Header | uint8[2] | 固定 `A5 5B` |
| 2 | 1 | Cmd | uint8 | 原命令号 |
| 3 | 1 | Result | uint8 | 执行结果 |
| 4 | 1 | Seq | uint8 | 原命令序号 |
| 5 | 2 | CRC16 | uint16 little-endian | CRC-16/MODBUS |
| 7 | 1 | Tail | uint8 | 固定 `5B` |

CRC 计算范围：

```text
Cmd[1] + Result[1] + Seq[1]
```

Result 定义：

| Result | 含义 |
|---:|---|
| `0x00` | 成功 |
| `0x01` | 设备未静止，拒绝执行 |
| `0x02` | 命令或参数错误 |
| `0x03` | 执行失败 |

## 角度清零

角度清零用于把当前积分角度置为 `0 deg`，不重新估计零偏。

命令：

```text
Cmd = 0x01
Param = 0x01
```

执行要求：

- 发送命令前保持 CY-Z 静止。
- 清零成功后，后续数据帧中的 `AngleDeg` 从 `0 deg` 附近重新积分。
- 如果设备未静止，返回 `Result=0x01`，角度不清零。

## 零偏校准

零偏校准用于重新估计陀螺仪静止零偏，并同时清除当前积分角度。

命令：

```text
Cmd = 0x01
Param = 0x02
```

执行要求：

- 校准期间必须保持 CY-Z 静止。
- 校准过程约持续 2 秒，期间数据输出可能短暂停顿。
- 校准成功后，后续角速度输出以新的零偏为基准，`AngleDeg` 从 `0 deg` 附近重新积分。
- 如果设备未静止或校准失败，返回对应错误码。

## 比例校准

比例校准用于修正角速度比例因子，使积分角度更接近实际旋转角度。

流程：

1. 保持 CY-Z 静止，发送开始比例校准命令 `Cmd=0x05`。
2. `Param` 选择校准圈数：`1`、`2`、`3` 或 `6`。
3. 收到成功 ACK 后，按选定圈数连续同方向旋转。
4. 旋转完成后停止并保持静止，发送完成比例校准命令 `Cmd=0x06, Param=0x00`。
5. CY-Z 根据累计角度计算并保存新的比例因子。
6. 建议发送 `Cmd=0x08, Param=0x00` 读取当前比例因子，确认保存结果。

计算逻辑：

```text
measured = abs(累计角度)
expected = 圈数 * 360.0
new_factor = old_factor * expected / measured
```

失败条件：

- 开始或完成校准时设备未静止。
- 测量角度明显偏离目标角度。
- 新比例因子超出允许范围。
- 校准过程中发送了取消命令 `Cmd=0x07`。

## 比例因子响应帧

读取比例因子命令 `Cmd=0x08` 返回固定 12 字节比例因子响应帧。

```text
A5 5C  Cmd[1]  Seq[1]  GyroScaleFactor[4]  Reserved[1]  CRC16[2]  5C
```

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---:|---:|---|---|---|
| 0 | 2 | Header | uint8[2] | 固定 `A5 5C` |
| 2 | 1 | Cmd | uint8 | 原命令号，当前为 `0x08` |
| 3 | 1 | Seq | uint8 | 原命令序号 |
| 4 | 4 | GyroScaleFactor | float32 little-endian | 当前角速度比例因子 |
| 8 | 1 | Reserved | uint8 | 固定 `0x00` |
| 9 | 2 | CRC16 | uint16 little-endian | CRC-16/MODBUS |
| 11 | 1 | Tail | uint8 | 固定 `5C` |

CRC 计算范围：

```text
Cmd[1] + Seq[1] + GyroScaleFactor[4] + Reserved[1]
```
