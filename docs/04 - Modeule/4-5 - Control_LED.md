# Control LED

# Controlling an LED

This guide demonstrates how to control an LED using the `led.py` routine, specifically implementing LED flashing via General Purpose Input/Output (GPIO) on the Jetson Orin Nano.

## Hardware Connection

Connect the LED's positive terminal to pin 12 of the Jetson Orin Nano and its negative terminal to pin 39 (GND). The LED's on/off state is controlled by setting the output level of pin 12 to high or low, respectively. The `time.sleep()` method from the `time` library is utilized to manage the interval between these state changes.

## Running the Program

To execute the program:

1.  Navigate to the `GPIO_test` directory:
    ```bash
    cd ~/GPIO_test
    ```
2.  Run the `led.py` script:
    ```bash
    sudo python3 led.py
    ```

## Expected Outcome

After connecting the hardware and executing the program, the LED will flash every two seconds.

**Important Note:**
When utilizing different LED modules, it is highly recommended to include an appropriately sized protective resistor in series between the LED and the Jetson Orin Nano's pins. This precaution prevents overcurrent, which could potentially damage the Jetson Orin Nano.

![](/img/docs/4-5/2023040400006.jpg)