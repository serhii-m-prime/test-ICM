# STM32 IMU FFT Analyzer

This project reads a 9-axis ICM-20948 sensor with an STM32F411CE, processes accelerometer samples using CMSIS-DSP FFT, and sends sensor and spectrum data to a computer over UART. A browser dashboard displays the measurements through the Web Serial API.

## Modules

- `imu_blackpill/Core/Src/main.c` — peripheral initialization, module instances, shared data structures, and module calls.
- `imu_blackpill/Core/Src/imu_sensor.c` — generic sensor interface for independently initialized accelerometer, gyroscope, and magnetometer components.
- `imu_blackpill/Core/Src/icm20948.c` — ICM-20948 implementation, including SPI communication and AK09916 magnetometer access through the internal I2C master.
- `imu_blackpill/Core/Src/fft_processor.c` — sample buffering, overlapping FFT windows, spectrum magnitude calculation, and peak detection.
- `imu_blackpill/Core/Src/uart_output.c` — JSON-compatible positional UART messages.
- `imu_blackpill/Core/Src/mavlink_service.c` — optional MAVLink component heartbeat service.
- `imu_blackpill/index.html` — Web Serial dashboard for sensor values, FFT spectrum, and raw messages.

Public module interfaces are located in `imu_blackpill/Core/Inc`.

## Data Flow

```text
ICM-20948
    |
    | accelerometer, gyroscope, magnetometer
    v
Timer sampling callback
    |
    +-----------------------------> latest 9-axis sensor data
    |
    +--> accelerometer Y samples --> overlapping sample buffer
                                      |
                                      v
                                  256-point FFT
                                      |
                                      +--> 128 spectrum bins
                                      +--> peak frequency and amplitude
                                      v
                              JSON-compatible UART output
                                      |
                                      v
                              Web Serial dashboard
```

## FFT Configuration

- Sampling frequency: approximately `1023.54 Hz`
- FFT size: `256` samples
- Window shift: `51` samples
- Frequency resolution: approximately `3.998 Hz/bin`
- Spectrum range: `0` to approximately `507.8 Hz`

## UART Protocol

Messages are positional JSON arrays:

```text
[0,"error description",[requestedComponents,initializedComponents]]
[1,[ax,ay,az],[gx,gy,gz],[mx,my,mz]]
[2,[peakFrequency,peakAmplitude,frequencyResolution],[bin0,...,bin127]]
[3,"status message"]
```

A missing or unavailable sensor component is represented by `null`.

## Project Configuration

The STM32CubeMX configuration is stored in `imu_blackpill/imu_blackpill.ioc`. The firmware project can be imported into STM32CubeIDE and built using its Debug configuration.

Application-level feature flags are stored in `imu_blackpill/Core/Inc/app_config.h`.
Set `APP_MAVLINK_ENABLED` to `0` to compile out MAVLink packet generation and
transmission, or to `1` to enable the component heartbeat service. The system ID,
component ID, heartbeat interval, and UART transmit timeout are configured in the
same file. USART2 pin and baud-rate configuration remains owned by STM32CubeMX.

When enabled, the MAVLink service requests `ATTITUDE` from the ArduPilot
component, publishes the latest magnetometer sample as `HIGHRES_IMU`, sends a
periodic `STATUSTEXT` containing roll/pitch/yaw, and maintains its component
`HEARTBEAT`.
