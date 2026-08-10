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
| 4    | brake    | uint8 | 0 .. 100 (%)             |
| 5    | buttons  | uint8 | bit0=reverse, bit1=armed |
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

### brake
* 0: Không phanh
* 1 .. 100: Lực phanh tương ứng (%)

### buttons
* bit0: reverse request (yêu cầu lùi)
* bit1: armed / enable drive (cho phép chạy)

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
3. Đặt brake = 100 (phanh khẩn cấp toàn lực)
4. Gửi CONTROL_CMD với buttons = 0 (disarmed)
5. Chuyển hệ thống sang trạng thái SAFE STOP