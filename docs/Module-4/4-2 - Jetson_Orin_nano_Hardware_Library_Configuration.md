# Jetson Orin nano Hardware Library Configuration

# Jetson Orin Nano Hardware Library Setup

## Hardware Library Configuration

### Introduction to Jetson.GPIO - Linux for Tegra

The Jetson TX1, TX2, AGX Xavier, Nano, and Orin series development boards are equipped with a 40-pin GPIO connector, similar to those found on Raspberry Pi devices. To manage the digital inputs and outputs of these GPIOs, you can utilize the Python library provided within the Jetson GPIO Library package. This library mirrors the API of Raspberry Pi's RPi.GPIO library, simplifying the process of migrating applications from Raspberry Pi to Jetson boards.

The Jetson GPIO library package includes the following components:

*   **`lib/Python/` subdirectory:** This directory contains the Python modules that implement all library functions. The primary module, `gpio.py`, is imported by applications and provides the necessary APIs. It directly imports `_event.py` and `_pin_data.py` for its functionality.
*   **`samples/` subdirectory:** This section offers sample applications designed to help users quickly become familiar with the library's API and begin development.
    *   `simple_input.py` and `simple_output.py` demonstrate basic read and write operations on GPIO pins.
    *   `button_led.py`, `button_event.py`, and `button_interrupt.py` illustrate how to control an LED with a button using busy waiting, blocking waiting, and interrupt callbacks, respectively.

This document will guide you through the contents of the Jetson GPIO library package, detail the necessary system configuration, and explain how to execute the provided sample applications and use the library's APIs. For comprehensive instructions on using the Jetson.GPIO library, please refer to the project's detailed documentation.

### Pin Diagram

#### GPIO and BCM Comparison Table

![](/img/docs/4-2/2023040400001.png)

### Environmental Setup

Follow these steps to configure your environment:

1.  **Download the Jetson-GPIO repository:**

    ```bash
    git clone https://github.com/NVIDIA/jetson-gpio
    ```

    ![](/img/docs/4-2/2023040400002.png)

2.  **Move the downloaded files to the `/opt/nvidia` directory:**

    If the `/opt/nvidia` directory already contains an existing `jetson-gpio` library, it is advisable to back it up first, as shown below:

    ![](/img/docs/4-2/2023040400003.png)

    After backing up, place the newly downloaded folder into `/opt/nvidia`. For example, if your downloaded `jetson-gpio` folder is in your home directory (`~/`), and you are currently in `/opt/nvidia`, you can move it using the following command:

    ```bash
    sudo mv ~/jetson-gpio ./
    ```

    ![](/img/docs/4-2/2023040400004.png)

3.  **Install the `pip3` tool:**

    ```bash
    sudo apt-get install python3-pip
    ```

4.  **Navigate to the Jetson GPIO library folder and install the library:**

    ```bash
    cd /opt/nvidia/jetson-gpio
    sudo python3 setup.py install
    ```

    ![](/img/docs/4-2/2023040400005.png)

5.  **Create a GPIO group, add your current user account to this group, and grant usage permissions:**

    ```bash
    sudo groupadd -f -r gpio
    sudo usermod -a -G gpio user_name
    ```

    ![](/img/docs/4-2/2023040400006.png)

    ```bash
    sudo cp /opt/nvidia/jetson-gpio/lib/python/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/
    ```

    To activate the new rule, either reboot your system or reload the udev rules with the following command:

    ```bash
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

    **Note:** Replace `user_name` with your actual system username (e.g., `jetson`).