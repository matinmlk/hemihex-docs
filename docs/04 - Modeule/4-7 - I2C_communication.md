# I2C communication

# I2C Communication

This guide outlines how to utilize I2C communication on the Jetson Orin Nano. Before using the I2C pins, the I2C service must be enabled. The I2C pins on the Jetson Orin Nano are illustrated below.

![](/img/docs/4-7/2023040600001.png)

## Installing I2C Tools

First, install the `i2c-tools` package. Open a terminal and execute the following commands:

```bash
sudo apt-get update
sudo apt-get install -y i2c-tools
```

To verify the installation status, run:

```bash
apt-cache policy i2c-tools
```

A successful installation will display output similar to this:

```
i2c-tools:
  Installed: 4.0-2
  Candidate: 4.0-2
  Version table:
 *** 4.0-2 500
        500 http://ports.ubuntu.com/ubuntu-ports bionic/universe arm64 Packages
        100 /var/lib/dpkg/status
```

## Detecting I2C Devices

You can scan for all I2C devices connected to a specific bus and display their I2C addresses. For instance, if an I2C device with address `0x0f` is connected to the I2C pins, its corresponding address will be shown in the output when you execute the following command:

```bash
sudo i2cdetect -y -r -a 7
```

![](/img/docs/4-7/2023040600002.png)

## Installing Python SMBus

SMBus is a Python library commonly used for I2C communication. If it's not already installed, use the following terminal commands:

```bash
sudo apt-get update
sudo apt-get install -y python3-smbus
```

The SMBus protocol offers a variety of library functions that facilitate I2C communication.

![](/img/docs/4-7/2023040600003.png)

## OLED Module Program Example

The following section presents a program example for an OLED module. For testing purposes, an compatible OLED module should be used.

![](/img/docs/4-7/2023040600004.png)

### Wiring Diagram

Connect the Jetson Orin Nano to the OLED module as follows:

*   Jetson Orin Nano pin 3 (SDA) → OLED SDA
*   Jetson Orin Nano pin 5 (SCL) → OLED SCL
*   Jetson Orin Nano pin 2 (5V) → OLED VCC
*   Jetson Orin Nano pin 6 (GND) → OLED GND

### Installing the Adafruit_SSD1306 Library

This OLED library is essential for working with the display. Install it using `pip`:

```bash
pip install Adafruit_SSD1306
```

![](/img/docs/4-7/2023040600005.png)

### OLED Initialization

The OLED module needs to be initialized.

![](/img/docs/4-7/2023040600006.png)

### Executing the Test Program

For those interested in exploring basic system information functions (such as local IP address, TF card space, memory usage, and system time), further details can be found within the `test_i2c_oled.py` file.

To run the test program, navigate to the `GPIO_test` directory and execute the script:

```bash
cd ~/GPIO_test
sudo python3 test_i2c_oled.py
```

If you are using a custom image, you may need to transfer the `data_i2c_Transfer oled.py` (referring to `test_i2c_oled.py` as indicated in the previous section) test file from the provided data attachments to your Jetson Orin Nano.

### Experimental Outcome

![](/img/docs/4-7/2023040600007.jpg)