# CAN Matrix

## Mục đích
Tài liệu định nghĩa toàn bộ bản tin CAN của dự án SmartCarSafe.
Tất cả ECU phải sử dụng đúng ID và layout dữ liệu trong tài liệu này.

---

## ECU trong hệ thống
| ECU     | Chức năng                   |
| ------- | --------------------------- |
| ECU1    | Sensor ECU                  |
| ECU2    | Actuator / Motor ECU        |
| ECU3    | Communication / Gateway ECU |
| Station | Trạm điều khiển RF + HMI    |

---

## CAN ID 0x100 — CONTROL_CMD

### Sender
ECU3 (Communication ECU)

### Receiver
ECU2 (Motor ECU)

### Chu kỳ
20 ms

### Payload (8 byte)
| Byte | Nội dung | Kiểu  | Giá trị          |
| ---- | -------- | ----- | ---------------- |
| 0    | throttle | int8  | -100 .. +100     |
| 1    | steering | int8  | -100 .. +100     |
| 2    | brake    | uint8 | 0 .. 100 (%)     |
| 3    | buttons  | uint8 | bit0=armed       |
| 4    | seq      | uint8 | sequence number  |
| 5    | reserved | uint8 | 0                |
| 6    | reserved | uint8 | 0                |
| 7    | crc      | uint8 | CRC8             |

---

## CAN ID 0x200 — SENSOR_DISTANCE

### Sender
ECU1 (Sensor ECU)

### Receiver
ECU2, ECU3

### Chu kỳ
50 ms

### Payload
| Byte | Nội dung    | Kiểu   | Đơn vị |
| ---- | ----------- | ------ | ------ |
| 0-1  | distance_mm | uint16 | mm     |
| 2-7  | reserved    |        |        |

---

## CAN ID 0x300 — TELEMETRY_STATUS

### Sender
ECU2 (Motor ECU)

### Receiver
ECU3 (Gateway ECU)

### Chu kỳ
100 ms

### Payload
| Byte | Nội dung        | Kiểu   | Đơn vị |
| ---- | --------------- | ------ | ------ |
| 0-1  | speed_rpm       | uint16 | RPM    |
| 2    | acc_active      | uint8  | 0/1    |
| 3    | aeb_active      | uint8  | 0/1    |
| 4    | battery_percent | uint8  | %      |
| 5-7  | reserved        |        |        |

---

## CAN ID 0x400 — SYSTEM_POST

### Sender
ECU3

### Receiver
Tất cả ECU

### Payload
| Byte | Nội dung |
| ---- | -------- |
| 0    | rf_ok    |
| 1    | can_ok   |
| 2    | spi_ok   |
| 3-7  | reserved |

---

## Ghi chú
* CAN tốc độ: **500 kbps**
* Dữ liệu signed dùng **two's complement**
* CRC dùng **CRC-8 polynomial 0x07**
* Tất cả bản tin đều dùng **Standard ID (11-bit)**