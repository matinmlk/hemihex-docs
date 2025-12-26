# External device serial communication

# Serial Communication with External Devices

This guide provides instructions for configuring and testing serial communication on the Jetson Orin Nano, including self-testing (loopback) and external communication.

## 1. Preparation and Loopback Test Setup

This section focuses on setting up a loopback test to verify the Jetson Orin Nano's serial port functionality (sending and receiving data on its own).

The TXD (Transmit Data) and RXD (Receive Data) pins for the serial port on the Jetson Orin Nano correspond to physical pins 8 and 10, respectively, as illustrated below.

![](/img/docs/4-6/2023041200001.png)

**Wiring for Loopback Test:**

To perform a loopback test, connect the TXD pin directly to the RXD pin:

*   Jetson Orin Nano Pin 8 (TXD) → Jetson Orin Nano Pin 10 (RXD)

![](/img/docs/4-6/2023041200002.png)

**Enable Serial Port Permissions:**

Before proceeding, grant the necessary permissions to access the serial port. Please note that this permission is temporary and will reset after a system shutdown or reboot; you will need to re-enable it each time.

```bash
sudo chmod 777 /dev/ttyTHS1
```

## 2. Testing with a Sample Python Script

This section demonstrates how to use a provided Python script to test the serial port's send and receive capabilities.

First, download the serial communication example script:

```bash
git clone https://github.com/JetsonHacksNano/UARTDemo
```

Navigate into the downloaded directory:

```bash
cd UARTDemo
```

If you do not intend to use the serial console (TTY) on UART, it is recommended to disable the serial console. This step is optional but may be necessary to prevent conflicts with your application.

```bash
systemctl stop nvgetty
systemctl disable nvgetty
udevadm trigger
```

Install the Python serial module, which is required by the example script:

```bash
sudo apt-get install python3-serial
```

Now, execute the Python script from your terminal:

```bash
sudo python3 uart_example.py
```

![](/img/docs/4-6/2023041200004.png)

Upon successful execution, you should observe a continuous loop of the string "NVIDIA Jetson Orin nano Developer Kit" being sent by the Jetson Orin Nano and echoed back to the terminal, confirming the serial port's functionality.

![](/img/docs/4-6/2023041200005.png)

## 3. Testing with a Graphical Serial Port Assistant (CuteCom)

For a graphical interface to interact with the serial port, you can use `cutecom`, a popular serial terminal program for Linux.

Install `cutecom` using the following command:

```bash
sudo apt install cutecom
```

Launch `cutecom` from the terminal:

```bash
sudo cutecom
```

![](/img/docs/4-6/2023041200003.png)

Once `cutecom` is open, typically no specific settings are required for a basic test. Simply click "Open" to activate the serial port connection. You can then type text into the "Input" field and press Enter to send the content through the serial port. The output will be displayed in the main window.

## 4. Testing with a Custom Python Script

This section outlines how to test the serial port using another specific Python script.

Navigate to the directory containing your script and execute it:

```bash
cd ~/GPIO_test
python3 test_serial_810.py
```

*(Note: If the `test_serial_810.py` program is not present, ensure it has been transferred to your Orin Nano system.)*

When this script runs, it typically demonstrates an echo function, where any message sent to the serial port is replied to and displayed.

## 5. Troubleshooting and Important Considerations

When communicating between a computer and the Jetson Orin Nano using a USB to TTL module, keep the following points in mind:

1.  **Cable Length:** Avoid using overly long DuPont cables, as this can introduce noise and lead to corrupted (garbled) data during transmission.
2.  **Voltage Supply:** If you experience issues where data is only being received but not sent, it might indicate insufficient voltage. Ensure the 5V output from your USB to TTL module is connected to a 5V input pin on the Jetson Orin Nano.
3.  **Serial Parameters:** If the wiring appears correct but you are still receiving garbled data, meticulously verify that the baud rate, parity, and stop bit settings are consistent across both communicating devices.