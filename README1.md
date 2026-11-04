# ACG525 Furuta Pendulum Bring-up

> 2026-10-23 弱外环试验版：角度参数保持 8/10/4，新位置/速度反馈已接入独立缩放、限幅、
> 变化率限制与平衡小角度区门控。新增诊断遥测用于分析漂移；实机稳定性尚待验证。当前参数、回退开关和测试顺序见
> [弱位置反馈说明](WEAK_POSITION_TUNING.md)；旧试调文档中的单角度参数属于历史版本。

This project targets the Gowin ACG525 board with the GW5A-LV25UG324ES FPGA.
It provides the first hardware bring-up path for the rotary inverted pendulum:

- MCP3201 SPI angle acquisition at 1 ksample/s
- Quadrature encoder position and speed capture
- 20 kHz dual-input PWM for the AT8236 motor driver
- A three-state safety/swing-up/balance controller with a position-hold outer loop
- Board OLED status display over I2C at `R3/T3`

The design starts in safe monitor mode and the OLED shows `SAFE`. `KEY0`
selects `RUN1` (low-speed forward test), `RUN2` (low-speed reverse test), or
`RUN3` (balance controller) while the motor remains stopped. P6-10 is the
start/pause/resume key: it starts the selected mode when stopped, pauses the
current mode when pressed during operation, and resumes that same mode when
pressed again if the selection is unchanged. `KEY0` is locked only while
running; it can change the selected mode while paused. P6-10 then starts the
new selection. Pause keeps the angle zero and position target; resuming RUN3
re-evaluates the pendulum state rather than freezing the control algorithm.
`KEY2` remains the active-low reset and emergency stop. `KEY1`
captures the current ADC value as the pendulum upright zero. `KEY2` is also the
active-low reset input.

## ACG525 connections

Use the left 40-pin GPIO connector (`P6`):

| P6 pin | Signal | FPGA pin | Connection |
|---:|---|---|---|
| 1 | GPIO0 | C17 | MCP3201 CS |
| 2 | GPIO1 | C18 | MCP3201 CLK |
| 4 | GPIO3 | G14 | MCP3201 OUT |
| 5 | GPIO4 | D17 | AT8236 AN1 |
| 6 | GPIO5 | D18 | AT8236 AN2 |
| 7 | GPIO6 | G16 | Encoder E1A |
| 8 | GPIO7 | G18 | Encoder E1B |
| 29 | 3.3V | - | MCP3201 VCC/REF and sensor VCC |
| 30 | GND | - | Common ground |

The board buttons are `KEY0=B16`, `KEY1=A15`, and `KEY2=C15`. The LEDs show:

- `LED0`: heartbeat
- `LED1`: ADC sampling activity
- `LED2`: balance mode (`1` means motor control is enabled)
- `LED3`: encoder activity

The external keys use P6-3=`F14` (`target_dec_n`, 减1), P6-9=`F17`
(`target_inc_n`, 加1), and P6-10=`F18` (`mode_start_n`, 启动/暂停). Each key
connects its FPGA pin to ground when pressed; the FPGA internal pull-up is
enabled. In balance mode, each position key changes the target by 100 encoder
counts. This step must be calibrated for the actual encoder and mechanism.

The board OLED uses the shared I2C bus: `OLED_SCL=R3`, `OLED_SDA=T3`.
The board UART transmit signal is `uart_tx=V8`; connect it to the USB-UART
receiver input. The current firmware sends a 44-byte diagnostic frame at
115200 baud, 8 data bits, no parity, 1 stop bit. It includes a control sample
ID, FPGA angle zero and error, position target, the angle PD term, applied
weak-position term, and pre-friction command. The host accepts this frame and
the earlier 14-byte format:

```text
55 AB VERSION LENGTH SAMPLE_ID[4] ADC[2] POS[4] VEL[2] CMD[2] MODE FLAGS
ANGLE_ZERO[2] ANGLE_ERROR[2] TARGET[4] ANGLE_PD[4] OUTER_APPLIED[4]
BALANCE_D[2] PRE_FRICTION[4] WINDOW_COUNT XOR
```

`XOR` covers bytes 2 through 42. The current version is telemetry-only;
`uart_rx=U8` is reserved for a later command protocol.
The display is initialized as an SSD1306-compatible 128x64 panel and refreshes
at approximately 1 Hz with the mode, ADC code, angle error, encoder speed,
and control command. The first line includes the selected/active mode and
`START` or `STOP`, for example `MODE:RUN1 START` or `MODE:RUN2 STOP`.
`START` means the mode is enabled, not that the motor necessarily has a
nonzero control command. The generated Gowin device configuration enables
`SSPI/I2C/CPU regular_io` because these two board pins are multiplexed with
configuration functions.
The generated Gowin device configuration also releases the CPU and I2C
alternate functions on these pins; no CPU soft/hard core is instantiated by
this project. SDA is open-drain bidirectional and SCL is released high between
low phases, so the board pull-ups must remain connected.

The board package and example sources used during pin verification are under
`D:\Gowin\ACG525`, especially the pin table in `04_引脚功能信息` and the
`02_例程源码\ch29_i2c_control.rar` I2C example.

## Bring-up sequence

1. Power the MCP3201 and the 103 sensor from 3.3 V and share ground with the
   FPGA and AT8236.
2. Program the design with the motor disconnected. Rotate the sensor and
   verify `LED1` activity.
3. Put the pendulum upright and press `KEY1` briefly to capture the zero. The
   filter accepts a press after four ADC sample ticks and emits one FPGA-clock
   pulse; the top level then captures the current ADC register value.
4. Connect the motor driver with the motor supply still disconnected. Press
   `KEY0` to select `RUN1`, then press P6-10 to start the low-speed forward
   test. Press P6-10 again to pause, use `KEY0` to select `RUN2`, and press
   P6-10 to start reverse. `KEY2` resets all settings to safe mode.
5. After direction is confirmed, select `RUN3`, place the pendulum near
   upright, and press P6-10 to start balance control. P6-9/P6-3 adjust the
   position target; P6-10 pauses/resumes, and `KEY2` resets to safe mode.
6. Tune the signed gains in `src/pendulum_controller.v` from measured response
   data.

The controller has three states: safe monitor, energy-like swing-up, and PD
balance. On entering RUN3 from another executed mode, the current encoder
position is latched as the position target. Resuming the same RUN3 keeps its
target. The swing-up and position gains are starting values only;
the angle scale, motor sign, `SWING_TARGET_ENERGY`, and `COS_ZERO_CROSS` must be
calibrated on the assembled mechanism before the motor is allowed to run freely.

The MCP3201 represents unsigned codes from 0 through 4095; an individual
motion does not need to hit both endpoints. A measured range of 6 through 4094
is near full scale and does not indicate truncated data. See
[diagnostic telemetry notes](DIAGNOSTIC_TELEMETRY.md) for the field meanings,
recording sessions, and test procedure.

## Controller bring-up

The MCP3201 receiver now captures rising edges 4 through 15, following
DS21290F Figure 6-1, and uses a 781.25 kHz SPI clock for the 3.3 V setup.
ADC telemetry reports the full 0..4095 electrical code range. The installed
SV01A103 sensor has about 333.3 degrees of effective travel, so the endpoints
are not treated as adjacent angles. Recalibrate the upright zero and host
reference after programming this version. Angle gains and thresholds
use the linear count scale; see
[ADC_TIMING_FIX.md](ADC_TIMING_FIX.md).

RUN3 balance output includes direction-specific friction compensation using
the measured +153/-150 startup commands. Compensation ramps from zero over
128 raw command counts and is included before the final +/-900 limit.
See [FRICTION_TUNING.md](FRICTION_TUNING.md) for the parameters and comparison
procedure. These are initial hardware tuning values, not a stability result.

1. Keep the motor supply disconnected and verify ADC and encoder activity.
2. Confirm that a positive angle error produces the expected motor direction at
   a very low PWM limit.  Reverse the sensor or motor sign in the controller if
   the correction drives the pendulum farther away.
3. Start with balance mode near upright. Tune `KP_ANGLE` until correction is
   visible, then increase `KD_ANGLE` until oscillation is damped.
4. Only after balance control is repeatable, tune `SWING_TARGET_ENERGY` and
   `SWING_GAIN` for automatic swing-up. The mode thresholds use raw ADC counts.
5. The position loop is intentionally weak. Increase `KP_POSITION` and
   `KD_POSITION` only after the angle loop is stable.

The UART telemetry interface is enabled on `uart_tx=V8`; `uart_rx=U8` remains
reserved for a later command protocol.

## Control regression

With ModelSim available, run `vsim -c -do "do tests/run_controls.do"` from the
project directory. The test covers mode locking while running, paused mode
selection, restart/resume, SAFE behavior, reset, position-target retention,
and OLED START/STOP text and glyphs. ADC samples and OLED idle state are
accelerated for this test; it does not validate ADC or I2C electrical timing
or the physical pendulum control response.
