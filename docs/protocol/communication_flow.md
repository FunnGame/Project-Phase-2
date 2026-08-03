# Communication Flow

## Manual Mode

Station
↓ RF
ECU3 (Gateway)
↓ CAN ID 0x100
ECU2 (Motor Control)
↓ PWM
Motor

---

## Telemetry

ECU2
↓ CAN ID 0x300
ECU3
↓ RF
Station HMI

---

## Sensor Data

ECU1
↓ CAN ID 0x200
ECU2 (ACC / AEB)
ECU3 (display + telemetry)

---

## Safe Stop

Nếu ECU3 mất RF quá 500 ms:

ECU3
↓ CAN ID 0x100 (throttle=0, steering=0, armed=0)
ECU2
↓ Stop motor
