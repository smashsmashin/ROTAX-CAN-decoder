# ROTAX-CAN-decoder

This project contains firmware for an ESP32-C3 (ESP32-C3-MINI-1 in particular) to initialize its CAN controller, receive CAN packets, and log them to the USB serial output. It also includes a status LED indication.

## Project Location

The source code for this project is located in [GitHub](https://github.com/smashsmashin/ROTAX-CAN-decoder), structured for the ESP-IDF.

## Hardware Requirements

*   An ESP32-C3-MINI-1 based board.
*   A CAN transceiver (e.g., SN65HVD230, TJA1050) correctly wired:
    *   ESP32 GPIO21 (TX) to CAN Transceiver TXD
    *   ESP32 GPIO20 (RX) to CAN Transceiver RXD
*   Status LED (optional) connected to ESP32 GPIO2 (with a current-limiting resistor to GND).
*   Appropriate CAN bus termination (typically 120 Ohm resistors at each end of the bus).
*   USB cable for flashing and serial monitoring.

## Software Requirements

*   Espressif IoT Development Framework (ESP-IDF) properly installed and configured. This project is intended for ESP-IDF v4.x or v5.x.
    *   Follow the official ESP-IDF Get Started Guide for your OS: [https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html)

## Configuration (in `main/can_logger.c`)

*   **CAN Pins:**
    *   `CAN_TX_PIN`: `GPIO_NUM_21`
    *   `CAN_RX_PIN`: `GPIO_NUM_20`
*   **Status LED Pin:**
    *   `STATUS_LED_GPIO`: `GPIO_NUM_2` (Example, change if different)
*   **CAN Baud Rate:** Default is 500kbps. This can be changed by modifying the `can_timing_config_t` structure. Comments in the code provide guidance on calculating values for `brp`, `tseg1`, and `tseg2`.

## Compilation and Flashing Instructions

1.  **Clone the Repository (if you haven't already) and Navigate to it:**
    ```bash
    # git clone <repository-url> # If not already done
    # cd <repository-name>       # If not already in the project root
    ```

2.  **Open a Terminal or Command Prompt:**
    *   Ensure you are in the root directory of this project (where the main `CMakeLists.txt` and the `main` folder are located).
    *   Activate the ESP-IDF environment. This command depends on your ESP-IDF installation path and OS:
        *   Linux/macOS: `source $HOME/esp/esp-idf/export.sh`
        *   Windows: `%USERPROFILE%\esp\esp-idf\export.bat`
        (Adjust the path `$HOME/esp/esp-idf` or `%USERPROFILE%\esp\esp-idf` if your IDF is installed elsewhere.)

3.  **Set Target Microcontroller:**
    ```bash
    idf.py set-target esp32c3
    ```

4.  **Configure Project (Optional but Recommended):**
    ```bash
    idf.py menuconfig
    ```
    *   Navigate to `Serial flasher config`.
    *   Set the `Default serial port` to the one your ESP32-C3 is connected to (e.g., `/dev/ttyUSB0` on Linux, `COM3` on Windows).
    *   You can also adjust the `Upload speed (baud rate)` if necessary.
    *   Save your configuration and exit `menuconfig`.

5.  **Build the Project:**
    ```bash
    idf.py build
    ```
    This command compiles the source code and links the firmware. If successful, it will create a `build` directory.

6.  **Flash the Firmware:**
    *   Ensure your ESP32-C3 board is connected to your computer via USB.
    *   Put the ESP32-C3 into bootloader mode. Many development boards do this automatically via the `idf.py` script. If yours doesn't, you typically need to:
        1.  Hold down the "BOOT" (or "FLASH") button.
        2.  While holding "BOOT", press and release the "RESET" (or "EN") button.
        3.  Release the "BOOT" button.
    *   Execute the flash command (replace `/dev/ttyUSB0` with your actual serial port if it's different):
        ```bash
        idf.py -p /dev/ttyUSB0 flash
        ```
        For Windows, it might be `idf.py -p COM3 flash`.

7.  **Monitor Serial Output:**
    *   Once flashing is complete, the ESP32-C3 should restart and begin running the CAN logger.
    *   To view the logs (including received CAN messages), use the IDF monitor tool:
        ```bash
        idf.py -p /dev/ttyUSB0 monitor
        ```
        (Replace `/dev/ttyUSB0` with your serial port).
    *   You should see initialization messages, and then any CAN packets received will be printed to the console. The status LED will also provide feedback on the CAN bus state.
    *   To exit the monitor, press `Ctrl+]`.

## Status LED Behavior (`STATUS_LED_GPIO`)

*   **Fast Blinking (100ms on/off):** Critical CAN initialization failure (driver install or start failed). Program execution halts in this state.
*   **LED OFF:** Normal operation. CAN bus is initialized and in a `CAN_STATE_RUNNING` state.
*   **LED ON (Solid):**
    *   CAN bus is in a `CAN_STATE_BUS_OFF` state.
    *   The system is currently attempting to recover from a `CAN_STATE_BUS_OFF` state.
    *   The system is in a `CAN_STATE_RECOVERING` state.
    *   If recovery to `CAN_STATE_RUNNING` is successful, the LED will turn OFF. If it remains `CAN_STATE_BUS_OFF` or `CAN_STATE_RECOVERING`, the LED stays ON.

## Troubleshooting

*   **Incorrect Baud Rate:** The default is 500kbps. If your CAN bus operates at a different speed (e.g., 125kbps, 250kbps, 1Mbps), you must modify the `can_timing_config_t` settings in `main/can_logger.c`.
*   **Wiring Issues:** Carefully verify all connections: ESP32 TX/RX to CAN transceiver, CAN transceiver to CAN bus (CAN_H, CAN_L), power, and ground. Ensure the CAN bus has proper termination resistors (usually 120 Ohms at each physical end of the bus).
*   **Serial Port Problems:** Ensure you've selected the correct serial port in `menuconfig` or with the `-p` option for `idf.py flash` and `idf.py monitor`. Check USB drivers if the port isn't detected.
*   **ESP-IDF Version Compatibility:** The code is written for ESP-IDF v4.x/v5.x. Older versions might have slight API differences.
*   **Build Failures:** Ensure ESP-IDF is correctly installed and its environment is active in your terminal. Check for any typos or errors reported by the compiler.
```
