# Audio Capture with ReSpeaker Microphone

## Learning Objectives

By the end of this lesson, you will be able to:

- Set up ReSpeaker USB microphone arrays for far-field audio capture
- Configure ALSA (Advanced Linux Sound Architecture) for low-latency recording
- Implement circular buffer audio streaming in Python
- Apply beamforming and noise cancellation for robot environments
- Integrate audio capture with ROS 2 for downstream speech processing

---

## Prerequisites

**Required Knowledge**:
- Basic Linux command-line operations
- Python programming (file I/O, threading)
- Understanding of audio concepts (sample rate, bit depth, channels)

**Required Hardware**:
- ReSpeaker Mic Array v2.0 (6-mic circular array) or 4-Mic Linear Array
- Host computer running Ubuntu 22.04
- USB 2.0 port (minimum)

**Required Software**:
- Ubuntu 22.04 (or compatible Linux distribution)
- Python 3.10+
- ROS 2 Humble (for integration exercises)

**Estimated Time**: 2-3 hours

---

## Introduction

Voice-controlled humanoid robots require robust far-field audio capture to understand commands in noisy environments. Unlike smartphone microphones designed for near-field (10-30cm) speech, robotics applications demand:

- **Far-field capture**: 1-5 meters from the speaker
- **Noise rejection**: Filtering motor noise, fans, ambient sounds
- **Directionality**: Beamforming to focus on speaker location
- **Low latency**: &lt;100ms for real-time interaction

The **Seeed Studio ReSpeaker** microphone arrays are purpose-built for these requirements, featuring:

- 6 or 4 MEMS microphones in circular/linear configuration
- Onboard XVSM-2000 DSP chip for beamforming
- USB audio device interface (plug-and-play)
- LED ring for visual feedback
- Support for 16 kHz sample rate (optimal for speech)

---

## Part 1: Hardware Setup

### Step 1: Verify ReSpeaker USB Connection

1. Connect ReSpeaker to a USB 2.0 port (USB 3.0 may cause compatibility issues)

2. Verify the device is detected:

```bash
lsusb | grep "2886:0018"
```

**Expected output**:
```
Bus 001 Device 005: ID 2886:0018 Seeed Technology Co., Ltd. ReSpeaker MicArray v2.0
```

3. Check ALSA device listing:

```bash
arecord -l
```

**Expected output**:
```
**** List of CAPTURE Hardware Devices ****
card 1: ArrayUAC10 [ReSpeaker 4 Mic Array (UAC1.0)], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

Note the **card number** (e.g., `card 1`) and **device number** (e.g., `device 0`).

---

### Step 2: Install ReSpeaker Python Library

The `respeaker` Python library provides access to the DSP chip for beamforming and LED control.

```bash
# Install system dependencies
sudo apt update
sudo apt install -y python3-pyaudio portaudio19-dev libatlas-base-dev

# Install Python libraries
pip3 install pyusb respeaker pixel-ring hid
```

**Verify installation**:

```bash
python3 -c "import usb.core; print('PyUSB OK')"
python3 -c "import pixel_ring; print('pixel_ring OK')"
```

---

### Step 3: Test LED Ring Control

The ReSpeaker's LED ring provides visual feedback (e.g., indicating listening state).

Create `test_led.py`:

```python
#!/usr/bin/env python3
import time
from pixel_ring import pixel_ring

# Initialize LED ring
pixel_ring.set_brightness(10)  # 0-100

# Test pattern: spinning blue light
print("Testing LED ring (Ctrl+C to stop)...")
try:
    while True:
        for i in range(12):  # 12 LEDs on v2.0
            pixel_ring.set_color_palette(i, 0x0000FF)  # Blue
            time.sleep(0.1)
            pixel_ring.set_color_palette(i, 0x000000)  # Off
except KeyboardInterrupt:
    pixel_ring.off()
    print("\nLED test complete")
```

Run:
```bash
chmod +x test_led.py
python3 test_led.py
```

You should see a blue light spinning around the ring.

---

## Part 2: Audio Recording with ALSA

### Step 1: Record Test Audio

Use ALSA's `arecord` to capture raw audio:

```bash
# Record 5 seconds of audio from ReSpeaker
# Replace hw:1,0 with your card:device from Step 1
arecord -D plughw:1,0 -f S16_LE -r 16000 -c 6 -d 5 test.wav

# Play back the recording
aplay test.wav
```

**Parameters explained**:
- `-D plughw:1,0`: Device (card 1, device 0)
- `-f S16_LE`: Format (16-bit signed little-endian)
- `-r 16000`: Sample rate (16 kHz - standard for speech)
- `-c 6`: Channels (6 microphones on v2.0, use `-c 4` for 4-mic version)
- `-d 5`: Duration (5 seconds)

---

### Step 2: Python Audio Capture

Create `capture_audio.py` for programmatic recording:

```python
#!/usr/bin/env python3
import pyaudio
import wave
import numpy as np

# Configuration
RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 6  # Change to 4 for 4-mic array
RESPEAKER_WIDTH = 2     # 2 bytes = 16-bit
RESPEAKER_INDEX = 1     # From arecord -l (card number)
CHUNK = 1024
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "output.wav"

# Initialize PyAudio
p = pyaudio.PyAudio()

# Open stream
stream = p.open(
    rate=RESPEAKER_RATE,
    format=p.get_format_from_width(RESPEAKER_WIDTH),
    channels=RESPEAKER_CHANNELS,
    input=True,
    input_device_index=RESPEAKER_INDEX,
    frames_per_buffer=CHUNK
)

print(f"Recording for {RECORD_SECONDS} seconds...")

frames = []

# Record audio
for i in range(0, int(RESPEAKER_RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("Recording complete")

# Stop stream
stream.stop_stream()
stream.close()
p.terminate()

# Save as WAV file
wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(RESPEAKER_CHANNELS)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames))
wf.close()

print(f"Saved to {WAVE_OUTPUT_FILENAME}")
```

Run:
```bash
python3 capture_audio.py
aplay output.wav
```

---

## Part 3: Beamforming and Noise Cancellation

### Step 1: Access DSP Parameters

The ReSpeaker's XVSM-2000 DSP chip provides advanced audio processing. Access it via `tuning.py`:

Create `get_dsp_params.py`:

```python
#!/usr/bin/env python3
from tuning import Tuning
import usb.core
import usb.util

# Find ReSpeaker device
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
if not dev:
    print("ReSpeaker not found!")
    exit(1)

# Initialize tuning interface
tuning = Tuning(dev)

# Read key DSP parameters
print(f"Direction of Arrival (DOA): {tuning.direction}")
print(f"Beamforming enabled: {tuning.read('STATNOISEONOFF')}")
print(f"Noise suppression level: {tuning.read('NONSTATNOISEONOFF')}")
print(f"Automatic Gain Control: {tuning.read('AGCONOFF')}")
```

**Download tuning.py**:
```bash
wget https://raw.githubusercontent.com/respeaker/usb_4_mic_array/master/tuning.py
```

Run:
```bash
python3 get_dsp_params.py
```

**Sample output**:
```
Direction of Arrival (DOA): 120
Beamforming enabled: 1
Noise suppression level: 1
Automatic Gain Control: 1
```

The **DOA (Direction of Arrival)** indicates the angle (0-360°) where the loudest sound is detected.

---

### Step 2: Enable Beamforming

Beamforming combines signals from all microphones to focus on a specific direction, reducing noise from other angles.

Create `enable_beamforming.py`:

```python
#!/usr/bin/env python3
from tuning import Tuning
import usb.core

dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
tuning = Tuning(dev)

# Enable stationary noise suppression (fans, motors)
tuning.write('STATNOISEONOFF', 1)

# Enable non-stationary noise suppression (background speech)
tuning.write('NONSTATNOISEONOFF', 1)

# Enable automatic gain control
tuning.write('AGCONOFF', 1)

# Set maximum gain (dB) - adjust based on environment
tuning.write('AGCMAXGAIN', 30)

# Set noise suppression level (0-3, higher = more aggressive)
tuning.write('SPEECHDETECTED', 2)

print("Beamforming and noise suppression enabled")
print(f"Current DOA: {tuning.direction}°")
```

Run before recording:
```bash
python3 enable_beamforming.py
python3 capture_audio.py
```

---

### Step 3: Real-Time DOA Visualization

Track speaker location in real-time:

```python
#!/usr/bin/env python3
from tuning import Tuning
from pixel_ring import pixel_ring
import usb.core
import time

dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
tuning = Tuning(dev)

pixel_ring.set_brightness(20)

print("Real-time DOA tracking (Ctrl+C to stop)...")
print("Speak to see the direction indicator")

try:
    while True:
        # Get direction of arrival (0-360°)
        direction = tuning.direction

        # Convert to LED index (0-11 for 12 LEDs)
        led_index = int((direction / 360.0) * 12) % 12

        # Light up the LED pointing to speaker
        pixel_ring.set_color_palette(led_index, 0x00FF00)  # Green
        time.sleep(0.1)
        pixel_ring.off()

except KeyboardInterrupt:
    pixel_ring.off()
    print("\nDOA tracking stopped")
```

Run:
```bash
python3 doa_tracking.py
```

Speak from different positions around the microphone - the green LED should point toward you.

---

## Part 4: ROS 2 Integration

### Step 1: Create Audio Publisher Node

Publish raw audio to a ROS 2 topic for processing by Whisper (next lesson).

Create `audio_publisher.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import pyaudio
import numpy as np

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')

        # ROS 2 publisher
        self.publisher_ = self.create_publisher(AudioData, 'audio/raw', 10)

        # Audio configuration
        self.RATE = 16000
        self.CHANNELS = 6
        self.CHUNK = 1024
        self.DEVICE_INDEX = 1

        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            rate=self.RATE,
            format=pyaudio.paInt16,
            channels=self.CHANNELS,
            input=True,
            input_device_index=self.DEVICE_INDEX,
            frames_per_buffer=self.CHUNK,
            stream_callback=self.audio_callback
        )

        self.get_logger().info('Audio publisher started')
        self.stream.start_stream()

    def audio_callback(self, in_data, frame_count, time_info, status):
        # Publish audio data
        msg = AudioData()
        msg.data = list(in_data)
        self.publisher_.publish(msg)

        return (in_data, pyaudio.paContinue)

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Step 2: Install audio_common_msgs

```bash
sudo apt install ros-humble-audio-common-msgs
```

---

### Step 3: Test Audio Stream

Terminal 1:
```bash
source /opt/ros/humble/setup.bash
python3 audio_publisher.py
```

Terminal 2:
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /audio/raw
```

**Expected output**:
```
average rate: 15.625
  min: 0.064s max: 0.064s std dev: 0.00000s window: 16
```

This confirms audio is streaming at ~16 Hz (1024 samples / 16000 Hz = 0.064s per chunk).

---

## Hands-On Exercise

### Exercise 1: Circular Buffer Recording

Implement a circular buffer to continuously record the last 5 seconds of audio (useful for "wake word" detection).

**Requirements**:
- Use `collections.deque` with `maxlen` for automatic buffer management
- Save buffer to file when a key is pressed
- Display buffer fill percentage

**Starter code**:

```python
import pyaudio
import wave
from collections import deque
import numpy as np
import threading
import sys
import termios
import tty

RATE = 16000
CHANNELS = 6
CHUNK = 1024
BUFFER_SECONDS = 5

# Calculate buffer size
buffer_size = int(RATE / CHUNK * BUFFER_SECONDS)
audio_buffer = deque(maxlen=buffer_size)

# TODO: Implement circular buffer recording
# 1. Continuously read from ReSpeaker
# 2. Append chunks to deque
# 3. On keypress, save deque contents to WAV
# 4. Display buffer fill %
```

**Solution approach**:
1. Use threading for non-blocking keyboard input
2. Convert deque to bytes with `b''.join(audio_buffer)`
3. Write to WAV file with `wave` module

---

### Exercise 2: Voice Activity Detection (VAD)

Implement simple energy-based VAD to detect when someone is speaking.

**Algorithm**:
1. Calculate RMS (root mean square) energy of each audio chunk
2. If energy > threshold, mark as "speech"
3. Use LED ring to indicate speech detection (green) vs. silence (red)

**Starter code**:

```python
def calculate_rms(audio_data):
    """Calculate RMS energy of audio chunk"""
    # TODO: Convert bytes to numpy array
    # TODO: Calculate RMS = sqrt(mean(samples^2))
    pass

def is_speech(rms_energy, threshold=500):
    """Determine if chunk contains speech"""
    # TODO: Compare energy to threshold
    pass

# Main loop
while True:
    data = stream.read(CHUNK)
    energy = calculate_rms(data)

    if is_speech(energy):
        # TODO: Set LED ring green
        pass
    else:
        # TODO: Set LED ring red
        pass
```

**Hint**: Use `np.frombuffer(data, dtype=np.int16)` to convert bytes to NumPy array.

---

## Troubleshooting

### Issue 1: "ReSpeaker not found" error

**Cause**: USB device not properly detected

**Solutions**:
1. Check USB connection (try a different port)
2. Verify with `lsusb | grep 2886:0018`
3. Add user to `audio` group:
   ```bash
   sudo usermod -a -G audio $USER
   ```
   Log out and back in for changes to take effect

4. Check permissions:
   ```bash
   ls -l /dev/snd/
   ```
   You should have read/write access to `pcmC*D*c` devices

---

### Issue 2: "Input overflowed" warning

**Cause**: System can't keep up with 16 kHz sampling rate (buffer overruns)

**Solutions**:
1. Increase `CHUNK` size (1024 → 2048)
2. Close resource-intensive applications
3. Use a dedicated USB 2.0 port (not a hub)
4. Increase process priority:
   ```bash
   sudo nice -n -10 python3 capture_audio.py
   ```

---

### Issue 3: Poor audio quality with noise

**Cause**: Beamforming not enabled or incorrect DSP settings

**Solutions**:
1. Run `enable_beamforming.py` before recording
2. Adjust noise suppression level (try 0, 1, 2, 3):
   ```python
   tuning.write('SPEECHDETECTED', 3)  # Max suppression
   ```
3. Check AGC gain (too high = amplified noise):
   ```python
   tuning.write('AGCMAXGAIN', 20)  # Lower gain
   ```

---

### Issue 4: LEDs not working

**Cause**: Missing `hidapi` library or permission issues

**Solutions**:
1. Install hidapi:
   ```bash
   sudo apt install libhidapi-libusb0 libhidapi-dev
   pip3 install hidapi
   ```

2. Add udev rule for USB access:
   ```bash
   echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"' | \
   sudo tee /etc/udev/rules.d/99-respeaker.rules
   sudo udevadm control --reload-rules
   ```
   Unplug and replug the ReSpeaker

---

## Performance Optimization

### Latency Reduction

For real-time applications (e.g., conversational AI), minimize latency:

1. **Reduce buffer size**:
   ```python
   CHUNK = 512  # 32ms latency at 16 kHz
   ```

2. **Use `stream_callback`** (non-blocking):
   ```python
   stream = p.open(..., stream_callback=audio_callback)
   ```

3. **Pin to CPU core**:
   ```bash
   taskset -c 0 python3 audio_publisher.py
   ```

**Latency budget**:
- Audio capture: 32 ms
- Beamforming (DSP): 10 ms
- ROS 2 publish: 5 ms
- **Total**: ~50 ms (acceptable for conversational AI)

---

### Multi-Microphone Processing

For advanced applications, process individual microphone channels separately:

```python
# Record all 6 channels
stream = p.open(..., channels=6)
data = stream.read(CHUNK)

# Convert to numpy array (interleaved samples)
samples = np.frombuffer(data, dtype=np.int16)

# De-interleave into separate channels
channels = samples.reshape(-1, 6).T  # Shape: (6, CHUNK)

# Access individual microphones
mic_0 = channels[0]  # First microphone
mic_1 = channels[1]  # Second microphone
# ... etc.

# Apply custom beamforming (e.g., delay-and-sum)
beamformed = np.mean(channels, axis=0)
```

---

## Summary

In this lesson, you learned to:

- ✅ Set up ReSpeaker USB microphone arrays for robotics
- ✅ Configure ALSA and PyAudio for 16 kHz, 6-channel recording
- ✅ Enable DSP-based beamforming and noise cancellation
- ✅ Implement real-time DOA tracking with LED feedback
- ✅ Integrate audio capture with ROS 2 for downstream processing

**Key Takeaways**:
- Far-field audio capture requires beamforming to reject noise
- The ReSpeaker's onboard DSP handles heavy lifting (no CPU overhead)
- 16 kHz sample rate is optimal for speech (balance quality vs. bandwidth)
- Circular buffers enable "always listening" with on-demand saving

---

## Additional Resources

### Official Documentation
- [ReSpeaker Mic Array v2.0 Wiki](https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)
- [PyAudio Documentation](https://people.csail.mit.edu/hubert/pyaudio/docs/)
- [ALSA Project](https://www.alsa-project.org/)

### Research Papers
- [Beamforming for Speech Recognition](https://ieeexplore.ieee.org/document/7471644) - Overview of beamforming techniques
- [Voice Activity Detection Survey](https://arxiv.org/abs/2011.01480) - Modern VAD approaches

### Code Examples
- [ReSpeaker Examples](https://github.com/respeaker/usb_4_mic_array/tree/master/examples) - Official example scripts
- [ROS 2 Audio Tools](https://github.com/ros-drivers/audio_common) - Audio common messages and utilities

### Video Tutorials
- [ReSpeaker Setup Tutorial](https://www.youtube.com/watch?v=_example) - Hardware setup walkthrough
- [Beamforming Explained](https://www.youtube.com/watch?v=_example) - Visual explanation of beamforming

---

## Next Lesson

In **Lesson 2: Speech-to-Text with Whisper**, you'll learn to:
- Run OpenAI's Whisper model for offline speech recognition
- Optimize Whisper for real-time inference on edge devices
- Integrate with ROS 2 for voice command processing
- Handle multilingual and noisy audio inputs

Continue to [Speech-to-Text with Whisper →](./whisper-api.md)
