# Pin reading function

# Pin Reading Function

This section outlines how to test the pin reading capabilities on the Jetson Orin Nano development kit using the provided `jetson-gpio` library routines. Once your environment is configured, you can proceed with these tests.

To access the sample programs, navigate to the following directory in your terminal:

```bash
cd /opt/nvidia/jetson-gpio/samples/
```

## Wiring Diagram

Refer to the diagram below for the necessary pin connections:

![](/img/docs/4-3/2023041500002.jpg)

## Example: `simple_input.py`

This program is a basic demonstration of digital input, configured to use the Broadcom (BCM) pin numbering scheme. It reads the current state of a specified pin and prints its value to the terminal. Specifically, it reads physical PIN12, which corresponds to BCM pin 18.

### Running the Program

Execute the program using the following command:

```bash
sudo python3 simple_input.py
```

### Expected Outcome

After launching the program, the terminal will display information regarding the pin's state. By default, without any external connection, BCM pin 18 (physical PIN12) will typically register a LOW value.

To observe a change:
*   Use a DuPont wire to connect physical PIN12 to a 3.3V source on your board. The terminal output should then indicate a HIGH value.
*   If connected to Ground (GND), the output will display LOW.

![](/img/docs/4-3/2023040400001.png)

### Important Considerations

*   **Pin Numbering:** It is essential to distinguish between the BCM pin numbering (e.g., BCM 18) and the physical pin numbering (e.g., PIN12, which is silk-screened on the board). The `simple_input.py` example reads BCM pin 18, which corresponds to physical PIN12.
*   **Voltage Levels:** The Jetson Orin Nano pins operate at a 3.3V logic level. To prevent potential hardware damage, avoid connecting them to 5V sources.