@echo off
set COM_PORT=%1

if "%COM_PORT%"=="" (
    set /p COM_PORT="Enter COM port (?? COM3): "
)

if "%COM_PORT%"=="" (
    echo 사용법:
    echo   esptool.exe --chip esp32 --port COMx --baud 921600 ...
    pause
    exit /b
)

echo Starting flash with port: %COM_PORT%
echo.

@echo esptool.exe --chip esp32 --port COM3 --baud 921600 --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x1000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin

@echo python -m esptool --chip esp32s3 --port %COM_PORT% --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0000 .pio/build/esp32_Debug/bootloader.bin 0x8000 .pio/build/esp32_Debug/partitions.bin 0xe000 C:\Users\ServerManager\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin 0x10000 .pio/build/esp32_Debug/firmware.bin

python -m esptool --chip esp32 --port %COM_PORT% --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x1000 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware_T53.bin
pause