# RF Protocol

## Mục đích

Giao thức truyền dữ liệu giữa Trạm điều khiển (Station) và ECU3 thông qua nRF24L01+.

---

## Tốc độ cập nhật

50 Hz (20 ms / frame)

---

## Kích thước frame

7 byte

---

## Layout frame

| Byte | Nội dung | Kiểu  | Giá trị                  |
| ---- | -------- | ----- | ------------------------ |
| 0    | magic    | uint8 | 0xA5                     |
| 1    | seq      | uint8 | 0..255                   |
| 2    | steering | int8  | -100 .. +100             |
| 3    | throttle | int8  | -100 .. +100             |
| 4    | mode     | uint8 | 0=MANUAL, 1=AUTO         |
| 5    | flags    | uint8 | bit0=reverse, bit1=armed |
| 6    | crc      | uint8 | CRC8                     |

---

## Ý nghĩa trường

### magic

Giúp đồng bộ frame và phát hiện dữ liệu rác.

### seq

Tăng sau mỗi frame để phát hiện mất gói hoặc trùng gói.

### steering

* -100: rẽ trái tối đa
* 0: thẳng
* +100: rẽ phải tối đa

### throttle

* -100: lùi tối đa
* 0: dừng
* +100: tiến tối đa

### mode

* 0: Manual
* 1: Auto

### flags

* bit0: reverse request
* bit1: armed / enable drive

---

## CRC

### Thuật toán

CRC-8 polynomial 0x07

### Dữ liệu tính CRC

Tính trên byte 0..5.

---

## Failsafe

Nếu ECU3 không nhận được frame hợp lệ trong **500 ms** thì phải:

1. Đặt throttle = 0
2. Đặt steering = 0
3. Gửi CONTROL_CMD với cờ disarmed
4. Chuyển hệ thống sang trạng thái SAFE STOP
