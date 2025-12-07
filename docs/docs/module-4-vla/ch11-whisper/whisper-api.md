# Speech-to-Text with OpenAI Whisper API

## Learning Objectives

By the end of this lesson, you will be able to:

- Set up OpenAI Whisper API for cloud-based speech recognition
- Configure authentication and API endpoints
- Implement audio transcription with language detection
- Handle streaming vs. batch transcription modes
- Optimize API usage for cost and latency
- Integrate Whisper transcriptions with ROS 2 voice command systems

---

## Prerequisites

**Required Knowledge**:
- Python programming (async/await, HTTP requests)
- Basic understanding of REST APIs
- Audio file formats (WAV, MP3, FLAC)
- Completion of [Lesson 1: Audio Capture](./audio-capture.md)

**Required Software**:
- Python 3.10+
- OpenAI Python SDK (`openai>=1.0.0`)
- ROS 2 Humble (for integration exercises)
- Audio files from previous lesson

**Required Accounts**:
- OpenAI API account with billing enabled
- API key with Whisper access

**Estimated Time**: 2-3 hours

---

## Introduction

OpenAI's **Whisper** is a state-of-the-art automatic speech recognition (ASR) model trained on 680,000 hours of multilingual data. Key advantages for robotics applications:

- **99 languages supported**: Multilingual voice commands for global deployment
- **Robust to noise**: Handles motor noise, background speech, accents
- **Automatic language detection**: No need to specify input language
- **Timestamped transcriptions**: Word-level timing for synchronization
- **Cloud-based**: No GPU required on robot (offload to OpenAI servers)

**Whisper API vs. Local Whisper**:

| Feature | Whisper API (Cloud) | Local Whisper (Edge) |
|---------|---------------------|----------------------|
| **Latency** | 500-2000ms (network) | 100-500ms (GPU-dependent) |
| **Cost** | $0.006/minute | Free (hardware cost) |
| **Accuracy** | Highest (large-v3 model) | Good (base/small models) |
| **Privacy** | Audio sent to OpenAI | Audio stays on device |
| **Hardware** | CPU-only robot | GPU required (CUDA) |
| **Reliability** | Depends on internet | Offline-capable |

**Recommended Use Case**: Use Whisper API for prototyping and cloud-connected robots. Switch to local Whisper for production deployments with strict latency/privacy requirements (covered in Lesson 3).

---

## Part 1: OpenAI API Setup

### Step 1: Create OpenAI Account and API Key

1. Sign up at [platform.openai.com](https://platform.openai.com/)

2. Navigate to **API Keys** section (https://platform.openai.com/api-keys)

3. Click **Create new secret key**:
   - Name: `whisper-robotics`
   - Permissions: `Whisper` access only (for security)

4. **Copy the API key** (you won't see it again):
   ```
   sk-proj-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
   ```

5. **Secure storage**: Never hardcode API keys in code!
   ```bash
   # Create .env file (add to .gitignore)
   echo "OPENAI_API_KEY=sk-proj-..." > .env
   ```

---

### Step 2: Install OpenAI Python SDK

```bash
# Install OpenAI SDK (version 1.0+ has updated API)
pip3 install openai python-dotenv

# Verify installation
python3 -c "import openai; print(f'OpenAI SDK v{openai.__version__}')"
```

**Expected output**:
```
OpenAI SDK v1.6.1
```

---

### Step 3: Test API Connectivity

Create `test_whisper_api.py`:

```python
#!/usr/bin/env python3
import os
from dotenv import load_dotenv
from openai import OpenAI

# Load environment variables from .env
load_dotenv()

# Initialize client
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Test with a simple audio file
audio_file_path = "test.wav"  # From previous lesson

try:
    with open(audio_file_path, "rb") as audio_file:
        # Transcribe audio
        transcription = client.audio.transcriptions.create(
            model="whisper-1",  # Current Whisper API model (large-v3)
            file=audio_file,
            response_format="text"
        )

    print(f"Transcription: {transcription}")

except FileNotFoundError:
    print(f"Error: {audio_file_path} not found")
    print("Record audio first with: arecord -D plughw:1,0 -f S16_LE -r 16000 -c 1 -d 5 test.wav")

except Exception as e:
    print(f"API Error: {e}")
```

Run:
```bash
python3 test_whisper_api.py
```

**Expected output**:
```
Transcription: Hello, this is a test of the Whisper speech recognition system.
```

**If you get an authentication error**:
- Verify API key is correct in `.env`
- Check billing is enabled on OpenAI account
- Ensure no typos in environment variable name

---

## Part 2: Transcription Modes and Parameters

### Step 1: Response Formats

Whisper API supports multiple output formats:

| Format | Output Type | Use Case |
|--------|-------------|----------|
| `text` | Plain string | Simple transcription |
| `json` | JSON object | Metadata (language, duration) |
| `verbose_json` | Detailed JSON | Word timestamps, confidence |
| `srt` | SubRip subtitles | Video captioning |
| `vtt` | WebVTT subtitles | Web video |

**Example: JSON format with metadata**

```python
transcription = client.audio.transcriptions.create(
    model="whisper-1",
    file=audio_file,
    response_format="json"
)

print(f"Text: {transcription.text}")
print(f"Language: {transcription.language}")  # Auto-detected
print(f"Duration: {transcription.duration}s")
```

**Sample output**:
```json
{
  "text": "Move forward three meters and turn left.",
  "language": "en",
  "duration": 2.5
}
```

---

### Step 2: Verbose JSON with Timestamps

For applications requiring word-level timing (e.g., lip-sync, subtitle alignment):

```python
transcription = client.audio.transcriptions.create(
    model="whisper-1",
    file=audio_file,
    response_format="verbose_json",
    timestamp_granularities=["word", "segment"]
)

# Access word-level timestamps
for word in transcription.words:
    print(f"{word.word}: {word.start}s - {word.end}s")
```

**Sample output**:
```
Move: 0.0s - 0.24s
forward: 0.24s - 0.56s
three: 0.56s - 0.84s
meters: 0.84s - 1.20s
and: 1.20s - 1.32s
turn: 1.32s - 1.60s
left: 1.60s - 1.88s
```

**Use case for robotics**: Synchronize robot actions with spoken commands (e.g., start moving when "forward" is detected, not after full sentence).

---

### Step 3: Language Detection and Translation

Whisper automatically detects the input language, but you can override:

```python
# Auto-detect language (default)
transcription = client.audio.transcriptions.create(
    model="whisper-1",
    file=audio_file
)

# Force specific language (improves accuracy if known)
transcription = client.audio.transcriptions.create(
    model="whisper-1",
    file=audio_file,
    language="es"  # ISO-639-1 code (es=Spanish, fr=French, etc.)
)

# Translate non-English to English
translation = client.audio.translations.create(
    model="whisper-1",
    file=audio_file  # Input: Spanish audio
    # Output: English text
)
```

**Example**:
- Input audio: "Muévete hacia adelante tres metros" (Spanish)
- `transcriptions.create()` → "Muévete hacia adelante tres metros"
- `translations.create()` → "Move forward three meters"

**Robotics use case**: Deploy same robot in multiple countries - translate voice commands to English for unified processing.

---

### Step 4: Prompt Engineering for Context

The `prompt` parameter provides context to improve transcription accuracy:

```python
# Without prompt (may mishear technical terms)
transcription = client.audio.transcriptions.create(
    model="whisper-1",
    file=audio_file
)
# Result: "Navigate to the ROS to topic" (incorrect)

# With prompt (provides domain-specific vocabulary)
transcription = client.audio.transcriptions.create(
    model="whisper-1",
    file=audio_file,
    prompt="This is a robotics command using ROS 2 navigation. Common terms: Nav2, ROS, LiDAR, SLAM, costmap."
)
# Result: "Navigate to the ROS 2 topic" (correct)
```

**Best practices**:
- Include domain-specific terms (ROS 2, Isaac Sim, Nav2, etc.)
- Add context about expected command structure
- Keep prompt under 224 tokens (~1000 characters)
- Update prompt based on observed errors

---

## Part 3: Production Implementation

### Step 1: Async Transcription for Low Latency

Blocking I/O hurts real-time performance. Use async/await:

```python
#!/usr/bin/env python3
import asyncio
import os
from dotenv import load_dotenv
from openai import AsyncOpenAI

load_dotenv()
client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

async def transcribe_audio(audio_path: str) -> str:
    """Asynchronously transcribe audio file"""
    try:
        with open(audio_path, "rb") as audio_file:
            transcription = await client.audio.transcriptions.create(
                model="whisper-1",
                file=audio_file,
                response_format="text",
                prompt="Robotics voice commands for ROS 2 navigation and manipulation."
            )
        return transcription
    except Exception as e:
        print(f"Transcription error: {e}")
        return ""

async def main():
    # Transcribe multiple files in parallel
    files = ["command1.wav", "command2.wav", "command3.wav"]
    tasks = [transcribe_audio(f) for f in files]

    # Wait for all transcriptions to complete
    results = await asyncio.gather(*tasks)

    for file, result in zip(files, results):
        print(f"{file}: {result}")

if __name__ == "__main__":
    asyncio.run(main())
```

**Performance**:
- Sequential (blocking): 3 files × 800ms = 2400ms total
- Parallel (async): max(800ms, 800ms, 800ms) = 800ms total
- **3x speedup** for batch processing

---

### Step 2: Error Handling and Retries

Handle network failures and rate limits:

```python
import time
from openai import OpenAI, APIError, RateLimitError, APIConnectionError

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def transcribe_with_retry(audio_path: str, max_retries: int = 3) -> str:
    """Transcribe with exponential backoff retry"""
    for attempt in range(max_retries):
        try:
            with open(audio_path, "rb") as audio_file:
                transcription = client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file,
                    response_format="text"
                )
            return transcription

        except RateLimitError:
            # Rate limit hit - wait and retry
            wait_time = 2 ** attempt  # Exponential backoff: 1s, 2s, 4s
            print(f"Rate limit hit. Retrying in {wait_time}s...")
            time.sleep(wait_time)

        except APIConnectionError as e:
            # Network issue - retry
            print(f"Connection error: {e}. Retrying...")
            time.sleep(1)

        except APIError as e:
            # API error (e.g., invalid file format)
            print(f"API error: {e}")
            return ""  # Don't retry for API errors

    print(f"Max retries ({max_retries}) exceeded")
    return ""

# Usage
result = transcribe_with_retry("command.wav")
```

**Error handling checklist**:
- ✅ Network failures → Retry with backoff
- ✅ Rate limits → Backoff and retry
- ✅ Invalid audio format → Log error, don't retry
- ✅ Authentication errors → Alert developer

---

### Step 3: Audio Format Optimization

Whisper API accepts multiple formats, but some are more efficient:

**Supported formats**: MP3, MP4, MPEG, MPGA, M4A, WAV, WEBM, FLAC

**Recommendations**:

| Format | Bitrate | File Size (5s) | Quality | Best For |
|--------|---------|----------------|---------|----------|
| **FLAC** | Lossless | 500 KB | Perfect | Archival |
| **WAV** | 256 kbps | 320 KB | Perfect | Development |
| **MP3** | 32 kbps | 20 KB | Good | Production |
| **OPUS** | 24 kbps | 15 KB | Good | Bandwidth-limited |

**Convert WAV to MP3 for production**:

```python
import subprocess

def convert_to_mp3(wav_path: str, mp3_path: str):
    """Convert WAV to MP3 with optimal settings for speech"""
    subprocess.run([
        "ffmpeg",
        "-i", wav_path,
        "-codec:a", "libmp3lame",
        "-b:a", "32k",  # 32 kbps (sufficient for speech)
        "-ar", "16000",  # 16 kHz sample rate
        "-ac", "1",      # Mono
        mp3_path
    ], check=True)

# Usage
convert_to_mp3("command.wav", "command.mp3")

# Transcribe compressed file (16x smaller)
with open("command.mp3", "rb") as audio_file:
    transcription = client.audio.transcriptions.create(
        model="whisper-1",
        file=audio_file
    )
```

**Cost savings**:
- WAV: 320 KB × $0.006/minute = $0.0032 per 5s
- MP3 (32kbps): 20 KB × $0.006/minute = $0.0002 per 5s
- **16x cost reduction** with negligible quality loss for speech

---

### Step 4: Streaming Audio Transcription

For real-time applications, transcribe audio chunks as they arrive:

```python
#!/usr/bin/env python3
import io
import wave
from openai import OpenAI
import pyaudio

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Audio configuration
CHUNK_DURATION = 3  # Transcribe every 3 seconds
RATE = 16000
CHUNK_SIZE = RATE * CHUNK_DURATION  # 48000 samples

# Initialize PyAudio
p = pyaudio.PyAudio()
stream = p.open(
    rate=RATE,
    format=pyaudio.paInt16,
    channels=1,
    input=True,
    input_device_index=1,  # ReSpeaker
    frames_per_buffer=1024
)

print("Streaming transcription (speak in 3-second chunks)...")

try:
    while True:
        # Record chunk
        frames = []
        for _ in range(0, int(RATE / 1024 * CHUNK_DURATION)):
            data = stream.read(1024)
            frames.append(data)

        # Create WAV in memory
        wav_buffer = io.BytesIO()
        with wave.open(wav_buffer, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))

        wav_buffer.seek(0)
        wav_buffer.name = "chunk.wav"  # Required by OpenAI SDK

        # Transcribe chunk
        transcription = client.audio.transcriptions.create(
            model="whisper-1",
            file=wav_buffer,
            response_format="text"
        )

        if transcription.strip():  # Only print non-empty
            print(f">> {transcription}")

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    stream.stop_stream()
    stream.close()
    p.terminate()
```

**Performance**:
- 3-second chunks → ~800ms transcription latency
- Total latency: 3000ms (record) + 800ms (transcribe) = **3.8s**
- Suitable for conversational AI (acceptable delay)

**Optimization**: Use 1-second chunks for lower latency (but higher API costs).

---

## Part 4: ROS 2 Integration

### Step 1: Create Whisper Transcription Node

Create `whisper_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import os
import io
import wave
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # OpenAI client
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # ROS 2 subscribers and publishers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio/chunks',  # From audio capture node
            self.audio_callback,
            10
        )

        self.transcript_pub = self.create_publisher(
            String,
            'voice/transcript',
            10
        )

        # Audio buffer
        self.audio_frames = []
        self.chunk_duration = 3.0  # seconds
        self.sample_rate = 16000
        self.max_frames = int(self.sample_rate / 1024 * self.chunk_duration)

        self.get_logger().info('Whisper node started')

    def audio_callback(self, msg: AudioData):
        """Accumulate audio chunks and transcribe when buffer is full"""
        self.audio_frames.append(bytes(msg.data))

        if len(self.audio_frames) >= self.max_frames:
            # Transcribe accumulated audio
            self.transcribe_buffer()
            self.audio_frames.clear()

    def transcribe_buffer(self):
        """Transcribe accumulated audio frames"""
        try:
            # Create WAV in memory
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(self.sample_rate)
                wf.writeframes(b''.join(self.audio_frames))

            wav_buffer.seek(0)
            wav_buffer.name = "chunk.wav"

            # Transcribe
            transcription = self.client.audio.transcriptions.create(
                model="whisper-1",
                file=wav_buffer,
                response_format="text",
                prompt="ROS 2 robotics voice commands for navigation and manipulation."
            )

            if transcription.strip():
                # Publish transcript
                msg = String()
                msg.data = transcription
                self.transcript_pub.publish(msg)
                self.get_logger().info(f'Transcript: {transcription}')

        except Exception as e:
            self.get_logger().error(f'Transcription failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

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

### Step 2: Test ROS 2 Integration

Terminal 1 (audio capture - from previous lesson):
```bash
source /opt/ros/humble/setup.bash
python3 audio_publisher.py
```

Terminal 2 (Whisper transcription):
```bash
source /opt/ros/humble/setup.bash
export OPENAI_API_KEY="sk-proj-..."
python3 whisper_node.py
```

Terminal 3 (monitor transcripts):
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /voice/transcript
```

**Expected output** (when you speak):
```
data: "Move forward three meters and turn left."
---
data: "Pick up the red cube on the table."
---
```

---

### Step 3: Launch File for Complete Pipeline

Create `whisper_pipeline.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Audio capture node
        Node(
            package='voice_control',
            executable='audio_publisher.py',
            name='audio_publisher',
            output='screen'
        ),

        # Whisper transcription node
        Node(
            package='voice_control',
            executable='whisper_node.py',
            name='whisper_node',
            output='screen',
            parameters=[{
                'chunk_duration': 3.0,
                'sample_rate': 16000
            }]
        ),

        # Command parser node (next lesson)
        Node(
            package='voice_control',
            executable='command_parser.py',
            name='command_parser',
            output='screen'
        )
    ])
```

Launch:
```bash
ros2 launch voice_control whisper_pipeline.launch.py
```

---

## Hands-On Exercise

### Exercise 1: Cost Monitoring

Implement a cost tracker to monitor Whisper API usage:

**Requirements**:
- Track total audio minutes transcribed
- Calculate cost at $0.006/minute
- Log daily usage to CSV file
- Alert when approaching budget threshold

**Starter code**:

```python
import csv
from datetime import datetime

class CostTracker:
    def __init__(self, budget_limit: float = 10.0):
        self.total_minutes = 0.0
        self.budget_limit = budget_limit  # USD
        self.cost_per_minute = 0.006
        self.log_file = "whisper_usage.csv"

    def log_transcription(self, duration_seconds: float):
        """Log transcription and update cost"""
        # TODO: Convert duration to minutes
        # TODO: Update total_minutes
        # TODO: Calculate current cost
        # TODO: Write to CSV (date, duration, cost)
        # TODO: Check if approaching budget limit (90%)
        pass

    def get_current_cost(self) -> float:
        """Calculate current month's cost"""
        # TODO: Return total_minutes * cost_per_minute
        pass

# Usage in WhisperNode
tracker = CostTracker(budget_limit=10.0)

# After each transcription
tracker.log_transcription(duration_seconds=3.0)
current_cost = tracker.get_current_cost()
```

**Bonus**: Create a ROS 2 service to query current cost.

---

### Exercise 2: Multilingual Command System

Extend the Whisper node to handle commands in multiple languages:

**Requirements**:
- Detect language of spoken command
- Translate non-English commands to English
- Route commands to appropriate language-specific handlers
- Support at least 3 languages (English, Spanish, Mandarin)

**Starter code**:

```python
def transcribe_multilingual(audio_file) -> dict:
    """Transcribe and detect language"""
    # TODO: Use response_format="verbose_json"
    # TODO: Extract language from response
    # TODO: If not English, call translations.create()
    # TODO: Return dict with original, translated, and language
    pass

# Expected return
{
    "original": "Muévete hacia adelante",
    "translated": "Move forward",
    "language": "es"
}
```

---

## Troubleshooting

### Issue 1: "Invalid file format" error

**Cause**: Audio file not in supported format or corrupted

**Solutions**:
1. Verify file format:
   ```bash
   file command.wav
   # Should show: RIFF (little-endian) data, WAVE audio
   ```

2. Check file size (must be < 25 MB):
   ```bash
   ls -lh command.wav
   ```

3. Re-encode with FFmpeg:
   ```bash
   ffmpeg -i command.wav -ar 16000 -ac 1 -c:a pcm_s16le fixed.wav
   ```

---

### Issue 2: High latency (>3 seconds)

**Cause**: Large audio files or network congestion

**Solutions**:
1. Reduce chunk duration (3s → 1s)
2. Compress audio (WAV → MP3):
   ```python
   # Use MP3 instead of WAV
   subprocess.run(["ffmpeg", "-i", "chunk.wav", "-b:a", "32k", "chunk.mp3"])
   ```

3. Use async API for parallel transcription
4. Check network latency:
   ```bash
   ping api.openai.com
   ```

---

### Issue 3: Poor transcription accuracy

**Cause**: Noisy audio, unclear speech, missing context

**Solutions**:
1. **Enable beamforming** (from previous lesson):
   ```python
   tuning.write('STATNOISEONOFF', 1)
   ```

2. **Add domain-specific prompt**:
   ```python
   prompt="ROS 2 navigation commands: Nav2, SLAM, costmap, LiDAR, move_base"
   ```

3. **Check audio quality**:
   ```bash
   aplay command.wav  # Should be clear, minimal noise
   ```

4. **Increase sample rate** (if using low-quality mic):
   ```python
   RATE = 24000  # Higher quality (but larger files)
   ```

---

### Issue 4: Rate limit errors

**Cause**: Exceeding 50 requests/minute (free tier) or 500 req/min (paid tier)

**Solutions**:
1. Implement request queue with rate limiting:
   ```python
   import time
   from collections import deque

   class RateLimiter:
       def __init__(self, max_requests: int = 50, window: int = 60):
           self.max_requests = max_requests
           self.window = window  # seconds
           self.requests = deque()

       def wait_if_needed(self):
           now = time.time()
           # Remove old requests outside window
           while self.requests and self.requests[0] < now - self.window:
               self.requests.popleft()

           # Check if at limit
           if len(self.requests) >= self.max_requests:
               sleep_time = self.requests[0] + self.window - now
               print(f"Rate limit reached. Waiting {sleep_time:.1f}s...")
               time.sleep(sleep_time)

           self.requests.append(now)

   # Usage
   limiter = RateLimiter(max_requests=50, window=60)

   limiter.wait_if_needed()
   transcription = client.audio.transcriptions.create(...)
   ```

2. Upgrade to paid tier for higher limits

---

## Cost Optimization Strategies

### Strategy 1: Silence Detection

Don't transcribe silent audio (saves API calls):

```python
import numpy as np

def is_silent(audio_data: bytes, threshold: int = 500) -> bool:
    """Detect if audio chunk is silent"""
    samples = np.frombuffer(audio_data, dtype=np.int16)
    rms = np.sqrt(np.mean(samples**2))
    return rms < threshold

# Only transcribe non-silent chunks
if not is_silent(audio_chunk):
    transcription = client.audio.transcriptions.create(...)
```

**Savings**: ~80% reduction in API calls for typical robot environments (lots of silence).

---

### Strategy 2: Wake Word Detection

Use local lightweight model to detect wake word ("Hey robot"), then use Whisper for full command:

```python
import pvporcupine

# Initialize Porcupine (local wake word detection)
porcupine = pvporcupine.create(
    access_key="YOUR_PICOVOICE_KEY",
    keywords=["jarvis", "alexa", "computer"]
)

# Listen for wake word (offline, free)
if porcupine.process(audio_frame) >= 0:
    print("Wake word detected! Activating Whisper...")

    # Now use Whisper for full command
    transcription = client.audio.transcriptions.create(...)
```

**Savings**: Only pay for commands after wake word (not continuous transcription).

---

### Strategy 3: Caching Frequent Commands

Cache transcriptions of common commands:

```python
import hashlib

class TranscriptionCache:
    def __init__(self):
        self.cache = {}

    def get_audio_hash(self, audio_data: bytes) -> str:
        """Create hash of audio data"""
        return hashlib.md5(audio_data).hexdigest()

    def get(self, audio_data: bytes) -> str | None:
        """Get cached transcription"""
        audio_hash = self.get_audio_hash(audio_data)
        return self.cache.get(audio_hash)

    def set(self, audio_data: bytes, transcription: str):
        """Cache transcription"""
        audio_hash = self.get_audio_hash(audio_data)
        self.cache[audio_hash] = transcription

# Usage
cache = TranscriptionCache()

cached = cache.get(audio_data)
if cached:
    print(f"Cache hit: {cached}")
    transcription = cached
else:
    transcription = client.audio.transcriptions.create(...)
    cache.set(audio_data, transcription)
```

**Savings**: ~20% reduction for robots with repetitive commands.

---

## Summary

In this lesson, you learned to:

- ✅ Set up OpenAI Whisper API with authentication and error handling
- ✅ Use multiple response formats (text, JSON, verbose JSON with timestamps)
- ✅ Implement async transcription for low-latency applications
- ✅ Optimize audio formats (WAV → MP3) for cost savings
- ✅ Integrate Whisper with ROS 2 for voice-controlled robotics
- ✅ Apply cost optimization strategies (silence detection, caching, wake words)

**Key Takeaways**:
- Whisper API is ideal for prototyping (99 languages, robust to noise)
- Use async API and MP3 compression for production systems
- Add domain-specific prompts to improve accuracy on technical terms
- Implement rate limiting and error handling for reliability
- Cost: ~$0.006/minute (~$0.36/hour of continuous listening)

---

## Additional Resources

### Official Documentation
- [OpenAI Whisper API Reference](https://platform.openai.com/docs/api-reference/audio)
- [Whisper Model Card](https://github.com/openai/whisper/blob/main/model-card.md)
- [OpenAI Python SDK](https://github.com/openai/openai-python)

### Research Papers
- [Robust Speech Recognition via Large-Scale Weak Supervision (Whisper Paper)](https://arxiv.org/abs/2212.04356)
- [Whisper: Robust Speech Recognition](https://cdn.openai.com/papers/whisper.pdf)

### Cost Calculators
- [OpenAI Pricing](https://openai.com/pricing)
- [Whisper Cost Calculator](https://whisper.ai/calculator) (unofficial)

### Alternative Solutions
- **Local Whisper**: [faster-whisper](https://github.com/guillaumekln/faster-whisper) (4x faster, GPU-accelerated)
- **Streaming ASR**: [Deepgram](https://deepgram.com/) (real-time streaming)
- **Offline ASR**: [Vosk](https://alphacephei.com/vosk/) (free, offline, 20+ languages)

### Video Tutorials
- [Whisper API Tutorial](https://www.youtube.com/watch?v=_example) - API setup walkthrough
- [Building Voice-Controlled Robots](https://www.youtube.com/watch?v=_example) - ROS 2 integration

---

## Next Lesson

In **Lesson 3: Speech-to-Text Pipeline**, you'll learn to:
- Run local Whisper models on edge devices (Jetson, Raspberry Pi)
- Optimize Whisper inference with TensorRT
- Build a complete transcription pipeline with error handling
- Compare cloud vs. edge ASR performance

Continue to [Speech-to-Text Pipeline →](./speech-to-text.md)
