# OrbitDrive_HALL_5

Orbit Drive **3.0** closed-loop FOC control (angle + estimated current + SpaceVectorPWM) with **analog hall** sensing and **HALL_2-style boot home**.

## What this is

| Piece | Source |
|-------|--------|
| Control / PID / serial+CAN protocol / EEPROM | OrbitDrive3.0 |
| Analog hall → GenericSensor | OrbitDrive_HALL_2 |
| Boot home (CW/CCW hard stops → mid = 0) + soft limits | OrbitDrive_HALL_2 |

No MT6835, SPI, SimpleFOCDrivers, or encoder EKF. EKF params remain in the protocol for compatibility; `applyTuning` ignores them.

## Boot sequence

1. Init hall pins + GenericSensor + driver  
2. Load EEPROM FOC calib if valid, else align  
3. `initFOC`  
4. `bootHomeCwCcwMid()` — soft travel limits from hard span  
5. Angle hold at 0°  
6. CAN + serial ready  

## Build

```bash
pio run
```

Board: STM32G431CBU6 (`stm32_g431`), USART1 on PB6/PB7.
