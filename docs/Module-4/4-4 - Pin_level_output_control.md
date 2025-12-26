# Pin level output control

# Pin Level Output Control

This section details the process of controlling the output logic level of a General Purpose Input/Output (GPIO) pin using Python scripts. The provided programs are designed to alternately set a specified physical pin to a high or low voltage state.

## Operation Overview

The `simple_out.py` and `test_pin.py` scripts are configured to toggle a designated physical pin's output between a high and low level every 2 seconds.

## Executing the Programs

To run the pin output control scripts, choose one of the following methods:

**Method 1: Using `simple_out.py` (controls PIN12)**

```bash
sudo python3 simple_out.py
```

**Method 2: Using `test_pin.py` (controls PIN21)**

```bash
cd ~/GPIO_test
python3 test_pin.py
```

## Verifying the Output

To confirm the desired behavior, use a voltmeter to measure the voltage at the controlled pin. You should observe the pin's voltage alternating between a high and low state.

*   The `simple_out.py` script operates on **PIN12**.
*   The `test_pin.py` script operates on **PIN21**.

The image below demonstrates the expected output behavior for **PIN21** when running the `test_pin.py` script.

![](/img/docs/4-4/2023040400001.png)