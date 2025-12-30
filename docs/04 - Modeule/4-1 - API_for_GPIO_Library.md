# API for GPIO Library

# API for GPIO Library

The Jetson GPIO library offers a comprehensive set of public APIs, mirroring those provided by the RPi.GPIO library. This document details the usage of each available API.

### 1. Importing the Library

To integrate the Jetson.GPIO module into your application, use the following import statement:

```python
import Jetson.GPIO as GPIO
```

This allows you to reference the module simply as `GPIO` throughout your code. For compatibility with existing RPi.GPIO applications, you can also import it using the `RPi.GPIO` alias:

```python
import RPi.GPIO as GPIO # Replaces Jetson.GPIO for existing RPi library code
```

### 2. Pin Numbering

The Jetson GPIO library supports four distinct methods for numbering IO pins. Two of these correspond to the modes found in the RPi.GPIO library:

*   **BOARD**: Refers to the physical pin numbers on the 40-pin GPIO connector.
*   **BCM**: References the Broadcom SoC GPIO numbers.

The other two modes, specific to Jetson, utilize string-based identifiers:

*   **CVM**: Corresponds to signal names on the CVM CVB connector.
*   **TEGRA_SOC**: Corresponds to signal names on the Tegra SoC.

You must explicitly set the desired pin numbering mode using one of the following calls:

```python
GPIO.setmode(GPIO.BOARD)    # Use physical board pin numbers
# or
GPIO.setmode(GPIO.BCM)      # Use Broadcom SoC GPIO numbers
# or
GPIO.setmode(GPIO.CVM)      # Use CVM connector signal names
# or
GPIO.setmode(GPIO.TEGRA_SOC)# Use Tegra SoC signal names
```

To verify the currently active mode, you can call:

```python
mode = GPIO.getmode()
```

The `mode` variable will return `GPIO.BOARD`, `GPIO.BCM`, `GPIO.CVM`, `GPIO.TEGRA_SOC`, or `None` if no mode has been set.

### 3. Warnings

The library will issue a warning if a GPIO pin you attempt to configure is already in use by an external process, especially if the new configuration deviates from its default input state. Warnings will also be generated if you try to clean up channels before both the mode and channel have been set. To suppress these warnings, use:

```python
GPIO.setwarnings(False)
```

### 4. Setting Channels

Before a GPIO channel can be used for input or output, it must first be configured.

To configure a channel as an input:

```python
GPIO.setup(channel, GPIO.IN)
```

To configure a channel as an output:

```python
GPIO.setup(channel, GPIO.OUT)
```

You can also specify an initial output value when setting up the channel:

```python
GPIO.setup(channel, GPIO.OUT, initial=GPIO.HIGH)
```

Multiple channels can be configured simultaneously as output:

```python
channels = [18, 12, 13]
GPIO.setup(channels, GPIO.OUT)
```

### 5. Input

To read the current value of a configured input channel:

```python
GPIO.input(channel)
```

This function returns either `GPIO.LOW` or `GPIO.HIGH`.

### 6. Output

To set the output state of a channel:

```python
GPIO.output(channel, state)
```

The `state` parameter can be `GPIO.LOW` or `GPIO.HIGH`.

You can also apply an output state to a list or tuple of channels:

```python
channels = [18, 12, 13] # or use a tuple
GPIO.output(channels, GPIO.HIGH) # Sets all specified channels to HIGH or GPIO.LOW
```

To set different states for multiple channels in a sequence:

```python
GPIO.output(channel_list_or_tuple, (GPIO.LOW, GPIO.HIGH, GPIO.HIGH)) # Example: Sets the first channel to LOW, the second and third to HIGH
```

### 7. Cleanup

It is good practice to reset all used GPIO pins to their default state at the end of your program. To clean up all channels that have been configured by the library:

```python
GPIO.cleanup()
```

If you prefer not to clear all channels, you can clean up specific individual channels or a list/tuple of channels:

```python
GPIO.cleanup(chan1)              # Cleans up only 'chan1'
GPIO.cleanup([chan1, chan2])     # Cleans up only 'chan1' and 'chan2'
GPIO.cleanup((chan1, chan2))     # Performs the same operation as the list example
```

### 8. Jetson Module Information and Library Version

To retrieve information about the Jetson module, access the `GPIO.JETSON_INFO` attribute:

```python
module_info = GPIO.JETSON_INFO
```

This provides a Python dictionary containing keys such as `P1_REVISION`, `RAM`, `REVISION`, `TYPE`, `MANUFACTURER`, and `PROCESS`. All values in this dictionary are strings, with the exception of `P1_REVISION`, which is an integer.

To obtain the library version information, access `GPIO.VERSION`:

```python
library_version = GPIO.VERSION
```

This attribute returns the library version as a string in `XYZ` format.

### 9. Interrupts

Beyond continuous polling, the library offers three methods for monitoring input events:

#### `wait_for_edge()`

This function halts the calling thread until a specified edge (rising, falling, or both) is detected on the given channel. It can be used as follows:

```python
GPIO.wait_for_edge(channel, GPIO.RISING)
```

The second parameter defines the edge to detect: `GPIO.RISING`, `GPIO.FALLING`, or `GPIO.BOTH`.

You can also set an optional timeout for the waiting period (in milliseconds):

```python
GPIO.wait_for_edge(channel, GPIO.RISING, timeout=500)
```

This function returns the channel number on which the edge was detected. If a timeout occurs before an edge is detected, it returns `None`.

#### `event_detected()`

This function allows you to periodically check if any events have occurred on a channel since its last call. First, set up event detection for the channel:

```python
# Set rising edge detection on the channel
GPIO.add_event_detect(channel, GPIO.RISING)

# Later in your code, to check for events:
run_other_code()
if GPIO.event_detected(channel):
    do_something()
```

As with `wait_for_edge()`, you can detect `GPIO.RISING`, `GPIO.FALLING`, or `GPIO.BOTH` edges.

#### Running a Callback Function on Edge Detection

This feature enables a secondary thread to execute a callback function concurrently with your main program when an edge is detected. This allows for asynchronous handling of GPIO events. Here's how to use it:

First, define your callback function:

```python
def callback_fn(channel):
    print("Callback called from channel %s" % channel)
```

Then, add event detection to the channel, specifying your callback function:

```python
# Add rising edge detection with a callback
GPIO.add_event_detect(channel, GPIO.RISING, callback=callback_fn)
```

If necessary, you can also specify a debounce time.

# API for GPIO Library

Multiple callback functions can also be assigned if necessary:
```python
def callback_one(channel):
    print("First Callback")

def callback_two(channel):
    print("Second Callback")

GPIO.add_event_detect(channel, GPIO.RISING)
GPIO.add_event_callback(channel, callback_one)
GPIO.add_event_callback(channel, callback_two)
```
In this scenario, these two callback functions will execute sequentially rather than concurrently, as a single thread handles all callback operations.

To prevent a single event from triggering callback functions multiple times (e.g., due to switch bounce), you can set a `bouncetime`. This parameter, specified in milliseconds, helps to consolidate closely spaced events into a single detection:
```python
# bouncetime is set in milliseconds
GPIO.add_event_detect(channel, GPIO.RISING, callback=callback_fn, bouncetime=200)
```
If edge detection is no longer required for a channel, it can be disabled as follows:
```python
GPIO.remove_event_detect(channel)
```

### 10. Checking GPIO Channel Functionality
This feature enables you to query the current operational mode of a given GPIO channel:
```python
GPIO.gpio_function(channel)
```
This function will return either `GPIO.IN` (indicating an input channel) or `GPIO.OUT` (indicating an output channel).

### 11. Pulse Width Modulation (PWM)
For detailed guidance on utilizing PWM channels, please refer to the `samples/simple_PWM.py` example.

It is crucial to note that this GPIO library provides PWM support exclusively for pins that incorporate hardware PWM controllers. Unlike some other GPIO libraries, this library does not implement software-simulated PWM.

Proper configuration of the system's pin multiplexer is essential to connect the hardware PWM controller to the relevant pins. If the pinmux is not configured correctly, the PWM signal will not reach the physical pin. This GPIO library does not dynamically alter pinmux configurations to achieve this. Please consult the L4T documentation for comprehensive information on configuring the pin multiplexer.