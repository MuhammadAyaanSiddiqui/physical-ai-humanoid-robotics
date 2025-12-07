# Local Speech-to-Text Pipeline with Whisper

## Learning Objectives

By the end of this lesson, you will be able to:

- Install and run OpenAI Whisper models locally (offline inference)
- Optimize Whisper for real-time performance on edge devices
- Build a complete STT pipeline with VAD, buffering, and post-processing
- Accelerate Whisper inference with faster-whisper (CTranslate2)
- Compare cloud API vs. local deployment trade-offs
- Implement robust error handling and fallback strategies

---

## Prerequisites

**Required Knowledge**:
- Python programming (threading, async/await)
- Audio processing basics (sample rates, formats)
- Completion of [Lesson 1: Audio Capture](./audio-capture.md)
- Completion of [Lesson 2: Whisper API](./whisper-api.md)

**Required Hardware**:
- **Minimum**: Intel Core i5 / AMD Ryzen 5 (CPU-only inference)
- **Recommended**: NVIDIA GPU with 4GB+ VRAM (GPU acceleration)
  - Jetson Nano: base/tiny models only
  - Jetson Xavier NX: small/medium models
  - RTX 3060+: All models including large-v3

**Required Software**:
- Ubuntu 22.04 or compatible Linux
- Python 3.10+
- CUDA Toolkit 11.8+ (for GPU acceleration)
- FFmpeg

**Estimated Time**: 3-4 hours

---

## Introduction

Running Whisper **locally** (on-device) provides several advantages over cloud API:

| Factor | Local Whisper | Cloud API (Lesson 2) |
|--------|--------------|----------------------|
| **Privacy** | Audio never leaves device | Audio sent to OpenAI |
| **Latency** | 100-500ms (GPU) | 500-2000ms (network) |
| **Cost** | Free (after hardware) | $0.006/minute |
| **Offline** | Works without internet | Requires connection |
| **Reliability** | No rate limits | 50 req/min (free tier) |
| **Hardware** | GPU recommended | CPU-only robot OK |

**When to use local Whisper**:
- Production deployments with strict latency requirements (&lt;500ms)
- Privacy-sensitive applications (healthcare, defense)
- Offline robots (warehouses without Wi-Fi)
- High-volume usage (>1000 hours/month → cheaper than API)

**When to use cloud API**:
- Prototyping and development
- CPU-only edge devices (no GPU)
- Low-volume usage (&lt;100 hours/month)
- Need for 99 language support with highest accuracy

---

## Part 1: Installing Local Whisper

### Step 1: Install OpenAI Whisper (Official)

```bash
# Install FFmpeg (audio processing)
sudo apt update
sudo apt install -y ffmpeg

# Install Whisper
pip3 install -U openai-whisper

# Verify installation
whisper --help
```

**Test with sample audio**:

```bash
# Download sample audio
wget https://github.com/openai/whisper/raw/main/tests/jfk.flac

# Transcribe with small model (244MB)
whisper jfk.flac --model small --language en

# Expected output:
# [00:00.000 --> 00:11.000] And so my fellow Americans, ask not what your country can do for you, ask what you can do for your country.
```

---

### Step 2: Whisper Model Selection

Whisper provides 5 model sizes with accuracy/speed trade-offs:

| Model | Parameters | Disk Size | VRAM | Relative Speed | WER (English) | Best For |
|-------|-----------|-----------|------|----------------|---------------|----------|
| **tiny** | 39M | 72 MB | 1 GB | 32x | 5.7% | Jetson Nano, prototyping |
| **base** | 74M | 140 MB | 1 GB | 16x | 4.3% | Edge devices (Pi 4) |
| **small** | 244M | 461 MB | 2 GB | 6x | 3.5% | **Recommended for robotics** |
| **medium** | 769M | 1.5 GB | 5 GB | 2x | 2.9% | High-accuracy applications |
| **large-v3** | 1550M | 2.9 GB | 10 GB | 1x | 2.4% | Cloud/workstation only |

**WER = Word Error Rate** (lower is better, 3.5% = 96.5% accuracy)

**Recommendation for humanoid robots**:
- **Jetson Nano**: `tiny` or `base` (CPU-only)
- **Jetson Xavier NX**: `small` (GPU-accelerated)
- **Laptop/Desktop with GPU**: `small` or `medium`
- **Cloud server**: `large-v3` (highest accuracy)

---

### Step 3: Download Models

Models are downloaded automatically on first use, but you can pre-download:

```bash
# Download specific model
python3 << EOF
import whisper
model = whisper.load_model("small")  # Downloads ~/.cache/whisper/small.pt
print(f"Model loaded: {model.dims}")
EOF
```

**Manual download** (for air-gapped systems):

```bash
# Download model weights
wget https://openaipublic.azureedge.net/main/whisper/models/9ecf779972d90ba49c06d968637d720dd632c55bbf19d441fb42bf17a411e794/small.pt

# Move to Whisper cache
mkdir -p ~/.cache/whisper
mv small.pt ~/.cache/whisper/
```

---

## Part 2: Python API for Local Inference

### Step 1: Basic Transcription

```python
#!/usr/bin/env python3
import whisper

# Load model (cached after first run)
model = whisper.load_model("small")

# Transcribe audio file
result = model.transcribe("audio.wav")

# Access transcription
print(f"Text: {result['text']}")
print(f"Language: {result['language']}")

# Access segments with timestamps
for segment in result['segments']:
    print(f"[{segment['start']:.2f}s -> {segment['end']:.2f}s] {segment['text']}")
```

**Sample output**:
```
Text: Move forward three meters and turn left.
Language: en
[0.00s -> 1.20s]  Move forward three meters
[1.20s -> 2.40s]  and turn left.
```

---

### Step 2: GPU Acceleration

By default, Whisper uses GPU if available. Verify:

```python
import whisper
import torch

# Check GPU availability
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"GPU: {torch.cuda.get_device_name(0)}" if torch.cuda.is_available() else "CPU-only")

# Load model (automatically uses GPU)
model = whisper.load_model("small")

# Check model device
print(f"Model device: {next(model.parameters()).device}")
# Output: cuda:0 (GPU) or cpu (CPU)

# Force CPU (for testing)
model_cpu = whisper.load_model("small", device="cpu")

# Force specific GPU
model_gpu1 = whisper.load_model("small", device="cuda:1")
```

**Performance comparison** (5-second audio):

| Device | Model | Inference Time | Real-Time Factor |
|--------|-------|----------------|------------------|
| Intel i7-12700 (CPU) | small | 2.5s | 0.5x (2x slower than real-time) |
| NVIDIA RTX 3060 (GPU) | small | 0.3s | 16x (16x faster than real-time) |
| Jetson Xavier NX (GPU) | small | 1.2s | 4x |
| Jetson Nano (CPU) | tiny | 6.0s | 0.8x |

**Real-Time Factor (RTF)**: `audio_duration / inference_time`
- RTF > 1.0 = Faster than real-time (good for robotics)
- RTF < 1.0 = Slower than real-time (not suitable for real-time applications)

---

### Step 3: Transcription Options

```python
result = model.transcribe(
    "audio.wav",

    # Language (auto-detect if omitted)
    language="en",  # ISO 639-1 code

    # Task: transcribe or translate
    task="transcribe",  # "translate" converts to English

    # Temperature for sampling (0.0 = greedy, higher = more creative)
    temperature=0.0,  # Recommended for robotics (deterministic)

    # Beam size (higher = more accurate but slower)
    beam_size=5,  # Default: 5

    # Best_of (number of candidates, higher = better quality)
    best_of=5,  # Default: 5

    # Compression ratio threshold (detect gibberish)
    compression_ratio_threshold=2.4,

    # Log probability threshold (filter low-confidence segments)
    logprob_threshold=-1.0,

    # No speech threshold (skip silent audio)
    no_speech_threshold=0.6,

    # Verbose (print progress)
    verbose=False
)
```

**Recommended settings for robotics**:

```python
# Fast, low-latency configuration
result = model.transcribe(
    "audio.wav",
    language="en",  # Skip language detection (saves 1s)
    temperature=0.0,  # Deterministic output
    beam_size=1,  # Greedy decoding (fastest)
    best_of=1,
    no_speech_threshold=0.6,  # Skip silence
    verbose=False
)

# High-accuracy configuration
result = model.transcribe(
    "audio.wav",
    temperature=0.0,
    beam_size=5,  # More thorough search
    best_of=5,
    compression_ratio_threshold=2.4,
    logprob_threshold=-1.0,
    verbose=True
)
```

---

## Part 3: Faster-Whisper (Optimized Implementation)

**faster-whisper** is a reimplementation using CTranslate2, achieving **4x speedup** with same accuracy.

### Step 1: Install faster-whisper

```bash
pip3 install faster-whisper

# For GPU acceleration, install CUDA-enabled version
pip3 install faster-whisper[gpu]
```

---

### Step 2: Basic Usage

```python
#!/usr/bin/env python3
from faster_whisper import WhisperModel

# Load model (automatically downloads if not cached)
model = WhisperModel(
    "small",
    device="cuda",  # or "cpu"
    compute_type="float16"  # or "int8" for quantization
)

# Transcribe
segments, info = model.transcribe(
    "audio.wav",
    language="en",
    beam_size=5
)

print(f"Detected language: {info.language} (probability: {info.language_probability:.2f})")

# Iterate through segments
for segment in segments:
    print(f"[{segment.start:.2f}s -> {segment.end:.2f}s] {segment.text}")
```

---

### Step 3: Quantization for Speed

Reduce model precision for faster inference:

| Compute Type | Precision | Speed | Accuracy | VRAM |
|--------------|-----------|-------|----------|------|
| **float32** | Full | 1x | 100% | 2 GB |
| **float16** | Half | 2x | 99.9% | 1 GB |
| **int8** | Quantized | 4x | 99.5% | 500 MB |

```python
# float16 (recommended for GPU)
model_fp16 = WhisperModel("small", device="cuda", compute_type="float16")

# int8 (CPU or low-VRAM GPUs)
model_int8 = WhisperModel("small", device="cpu", compute_type="int8")

# Benchmark
import time

audio_path = "test.wav"

# float16
start = time.time()
segments, _ = model_fp16.transcribe(audio_path)
list(segments)  # Force evaluation
print(f"float16: {time.time() - start:.2f}s")

# int8
start = time.time()
segments, _ = model_int8.transcribe(audio_path)
list(segments)
print(f"int8: {time.time() - start:.2f}s")
```

**Typical results (5s audio, RTX 3060)**:
```
float16: 0.28s
int8: 0.15s (2x faster, negligible accuracy loss)
```

---

### Step 4: Batched Inference (Advanced)

Process multiple audio files in parallel:

```python
from faster_whisper import WhisperModel
import concurrent.futures

model = WhisperModel("small", device="cuda", compute_type="float16")

def transcribe_file(audio_path: str) -> str:
    """Transcribe single file"""
    segments, _ = model.transcribe(audio_path, beam_size=1)
    return " ".join([seg.text for seg in segments])

# Transcribe multiple files in parallel
audio_files = ["cmd1.wav", "cmd2.wav", "cmd3.wav", "cmd4.wav"]

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
    results = list(executor.map(transcribe_file, audio_files))

for file, transcript in zip(audio_files, results):
    print(f"{file}: {transcript}")
```

---

## Part 4: Complete STT Pipeline

Build a production-ready pipeline with VAD, buffering, and error handling.

### Step 1: Voice Activity Detection (VAD)

Use Silero VAD to detect speech before transcription (saves compute):

```bash
pip3 install silero-vad
```

```python
#!/usr/bin/env python3
import torch
torch.set_num_threads(1)  # Reduce CPU usage

# Load Silero VAD model
model, utils = torch.hub.load(
    repo_or_dir='snakers4/silero-vad',
    model='silero_vad',
    force_reload=False
)

(get_speech_timestamps, _, read_audio, *_) = utils

def detect_speech(audio_path: str) -> bool:
    """Return True if audio contains speech"""
    wav = read_audio(audio_path)
    speech_timestamps = get_speech_timestamps(
        wav,
        model,
        threshold=0.5,  # Sensitivity (0.0-1.0)
        min_speech_duration_ms=250,  # Minimum speech length
        min_silence_duration_ms=100
    )
    return len(speech_timestamps) > 0

# Usage
if detect_speech("audio.wav"):
    print("Speech detected - transcribing...")
    # Run Whisper
else:
    print("No speech detected - skipping")
```

**Performance impact**:
- VAD inference: ~10ms (on CPU)
- Whisper inference: ~300ms (on GPU)
- **Savings**: Skip 90% of silent audio (motors running, no speech)

---

### Step 2: Ring Buffer for Continuous Recording

Implement "always listening" system that saves last N seconds on trigger:

```python
#!/usr/bin/env python3
import pyaudio
import wave
from collections import deque
import threading
import time

class AudioRingBuffer:
    def __init__(self, duration_seconds: int = 5, sample_rate: int = 16000):
        self.duration = duration_seconds
        self.sample_rate = sample_rate
        self.chunk_size = 1024

        # Ring buffer (automatically discards old data)
        buffer_size = int(sample_rate / self.chunk_size * duration_seconds)
        self.buffer = deque(maxlen=buffer_size)

        # Recording state
        self.is_recording = False
        self.p = pyaudio.PyAudio()
        self.stream = None

    def start(self):
        """Start continuous recording"""
        self.is_recording = True
        self.stream = self.p.open(
            rate=self.sample_rate,
            format=pyaudio.paInt16,
            channels=1,
            input=True,
            input_device_index=1,  # ReSpeaker
            frames_per_buffer=self.chunk_size,
            stream_callback=self._audio_callback
        )
        self.stream.start_stream()
        print(f"Recording started (buffering last {self.duration}s)")

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """Callback for incoming audio"""
        if self.is_recording:
            self.buffer.append(in_data)
        return (in_data, pyaudio.paContinue)

    def save_buffer(self, filename: str):
        """Save ring buffer to WAV file"""
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(b''.join(self.buffer))
        print(f"Saved {len(self.buffer)} chunks to {filename}")

    def stop(self):
        """Stop recording"""
        self.is_recording = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.p.terminate()

# Usage example
buffer = AudioRingBuffer(duration_seconds=5)
buffer.start()

try:
    while True:
        command = input("Press Enter to save buffer, or 'q' to quit: ")
        if command == 'q':
            break
        buffer.save_buffer(f"capture_{int(time.time())}.wav")
except KeyboardInterrupt:
    pass
finally:
    buffer.stop()
```

**Use case**: Robot continuously buffers audio. When wake word is detected, save last 5 seconds and transcribe.

---

### Step 3: Complete Pipeline with Error Handling

```python
#!/usr/bin/env python3
from faster_whisper import WhisperModel
import torch
import wave
import time
from pathlib import Path

class RobustWhisperSTT:
    def __init__(self, model_size: str = "small", device: str = "cuda"):
        # Initialize Whisper
        self.whisper = WhisperModel(
            model_size,
            device=device,
            compute_type="float16" if device == "cuda" else "int8"
        )

        # Initialize VAD
        self.vad_model, utils = torch.hub.load(
            'snakers4/silero-vad', 'silero_vad', force_reload=False
        )
        self.get_speech_timestamps, _, self.read_audio, *_ = utils

        print(f"Whisper STT initialized (model={model_size}, device={device})")

    def has_speech(self, audio_path: str) -> bool:
        """Check if audio contains speech using VAD"""
        try:
            wav = self.read_audio(audio_path)
            speech_timestamps = self.get_speech_timestamps(
                wav, self.vad_model, threshold=0.5
            )
            return len(speech_timestamps) > 0
        except Exception as e:
            print(f"VAD error: {e}")
            return True  # Assume speech on error (fail-safe)

    def transcribe(self, audio_path: str, language: str = "en") -> dict:
        """
        Transcribe audio file with error handling

        Returns:
            dict: {
                "success": bool,
                "text": str,
                "language": str,
                "duration": float,
                "inference_time": float,
                "error": str | None
            }
        """
        result = {
            "success": False,
            "text": "",
            "language": language,
            "duration": 0.0,
            "inference_time": 0.0,
            "error": None
        }

        try:
            # Validate file exists
            audio_file = Path(audio_path)
            if not audio_file.exists():
                raise FileNotFoundError(f"Audio file not found: {audio_path}")

            # Get audio duration
            with wave.open(str(audio_file), 'rb') as wf:
                frames = wf.getnframes()
                rate = wf.getframerate()
                result["duration"] = frames / float(rate)

            # Check for speech
            if not self.has_speech(audio_path):
                result["error"] = "No speech detected"
                return result

            # Transcribe with timing
            start_time = time.time()
            segments, info = self.whisper.transcribe(
                audio_path,
                language=language,
                beam_size=1,  # Fast decoding
                vad_filter=True,  # Additional VAD filtering
                vad_parameters=dict(threshold=0.5)
            )

            # Combine segments
            text = " ".join([seg.text.strip() for seg in segments])
            result["inference_time"] = time.time() - start_time

            # Check for empty transcription
            if not text.strip():
                result["error"] = "Empty transcription"
                return result

            # Success
            result["success"] = True
            result["text"] = text
            result["language"] = info.language

        except FileNotFoundError as e:
            result["error"] = str(e)
        except Exception as e:
            result["error"] = f"Transcription failed: {e}"

        return result

# Usage
stt = RobustWhisperSTT(model_size="small", device="cuda")

# Transcribe single file
result = stt.transcribe("command.wav", language="en")

if result["success"]:
    print(f"Transcript: {result['text']}")
    print(f"Inference time: {result['inference_time']:.2f}s")
    print(f"Real-time factor: {result['duration'] / result['inference_time']:.1f}x")
else:
    print(f"Error: {result['error']}")
```

---

### Step 4: ROS 2 Integration

Create a ROS 2 node for local Whisper:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import wave
import tempfile
from robust_whisper_stt import RobustWhisperSTT

class LocalWhisperNode(Node):
    def __init__(self):
        super().__init__('local_whisper_node')

        # Initialize Whisper
        self.stt = RobustWhisperSTT(model_size="small", device="cuda")

        # ROS 2 subscribers and publishers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio/chunks',
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
        self.chunk_duration = 3.0
        self.sample_rate = 16000
        self.max_frames = int(self.sample_rate / 1024 * self.chunk_duration)

        self.get_logger().info('Local Whisper node started')

    def audio_callback(self, msg: AudioData):
        """Accumulate audio and transcribe when buffer is full"""
        self.audio_frames.append(bytes(msg.data))

        if len(self.audio_frames) >= self.max_frames:
            self.transcribe_buffer()
            self.audio_frames.clear()

    def transcribe_buffer(self):
        """Transcribe accumulated audio"""
        # Save to temporary WAV file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
            with wave.open(tmp.name, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(self.sample_rate)
                wf.writeframes(b''.join(self.audio_frames))

            # Transcribe
            result = self.stt.transcribe(tmp.name, language="en")

            if result["success"]:
                # Publish transcript
                msg = String()
                msg.data = result["text"]
                self.transcript_pub.publish(msg)

                self.get_logger().info(
                    f'Transcript: {result["text"]} '
                    f'({result["inference_time"]:.2f}s)'
                )
            else:
                self.get_logger().warn(f'Transcription failed: {result["error"]}')

def main(args=None):
    rclpy.init(args=args)
    node = LocalWhisperNode()

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

## Hands-On Exercise

### Exercise 1: Latency Benchmark

Compare different Whisper configurations:

**Requirements**:
- Test 5 audio files (1s, 3s, 5s, 10s, 30s)
- Measure inference time for each model size (tiny, base, small, medium)
- Calculate real-time factor (RTF)
- Generate comparison table

**Starter code**:

```python
import time
from faster_whisper import WhisperModel

models = ["tiny", "base", "small", "medium"]
audio_files = ["1s.wav", "3s.wav", "5s.wav", "10s.wav", "30s.wav"]

results = {}

for model_name in models:
    # TODO: Load model
    # TODO: For each audio file:
    #   - Transcribe and measure time
    #   - Calculate RTF = audio_duration / inference_time
    # TODO: Store results in dictionary
    pass

# TODO: Print comparison table
```

**Expected output format**:
```
| Model  | 1s    | 3s    | 5s    | 10s   | 30s   | Avg RTF |
|--------|-------|-------|-------|-------|-------|---------|
| tiny   | 24x   | 18x   | 15x   | 12x   | 10x   | 15.8x   |
| base   | 18x   | 14x   | 12x   | 10x   | 8x    | 12.4x   |
| small  | 12x   | 10x   | 8x    | 7x    | 6x    | 8.6x    |
| medium | 6x    | 5x    | 4x    | 3.5x  | 3x    | 4.3x    |
```

---

### Exercise 2: Hybrid Cloud/Local System

Implement a system that uses local Whisper for low-latency, falls back to cloud API for failures:

**Requirements**:
- Try local Whisper first (timeout: 2 seconds)
- If local fails or times out, use OpenAI API
- Log which method was used for each transcription
- Track success rate and average latency for each method

**Starter code**:

```python
from faster_whisper import WhisperModel
from openai import OpenAI
import os
import time

class HybridWhisperSTT:
    def __init__(self):
        # TODO: Initialize local Whisper
        # TODO: Initialize OpenAI client
        self.stats = {"local": [], "cloud": []}

    def transcribe(self, audio_path: str, timeout: float = 2.0) -> dict:
        """Try local first, fallback to cloud"""
        # TODO: Try local Whisper with timeout
        # TODO: On failure/timeout, use cloud API
        # TODO: Log which method was used + latency
        pass

# Usage
hybrid = HybridWhisperSTT()
result = hybrid.transcribe("command.wav")
print(f"Method: {result['method']}, Latency: {result['latency']:.2f}s")
```

---

## Troubleshooting

### Issue 1: "CUDA out of memory" error

**Cause**: Model too large for GPU VRAM

**Solutions**:
1. Use smaller model:
   ```python
   model = WhisperModel("base")  # Instead of "small"
   ```

2. Use int8 quantization:
   ```python
   model = WhisperModel("small", compute_type="int8")
   ```

3. Use CPU:
   ```python
   model = WhisperModel("small", device="cpu", compute_type="int8")
   ```

4. Clear GPU cache:
   ```python
   import torch
   torch.cuda.empty_cache()
   ```

---

### Issue 2: Slow inference on CPU

**Cause**: CPU inference is 10-20x slower than GPU

**Solutions**:
1. Use `tiny` or `base` model (faster on CPU)
2. Enable int8 quantization:
   ```python
   model = WhisperModel("tiny", device="cpu", compute_type="int8")
   ```

3. Reduce beam size:
   ```python
   segments, _ = model.transcribe(audio, beam_size=1)  # Greedy decoding
   ```

4. Increase chunk duration (transcribe longer segments less frequently):
   ```python
   CHUNK_DURATION = 5  # seconds (instead of 3)
   ```

---

### Issue 3: Poor accuracy on technical terms

**Cause**: Whisper not trained on robotics terminology

**Solutions**:
1. Add initial prompt with technical terms:
   ```python
   segments, _ = model.transcribe(
       audio,
       initial_prompt="Robotics commands: ROS 2, Nav2, LiDAR, SLAM, Isaac Sim"
   )
   ```

2. Post-process transcription with keyword replacement:
   ```python
   replacements = {
       "ross too": "ROS 2",
       "lie dar": "LiDAR",
       "slam": "SLAM",
       "nav too": "Nav2"
   }

   transcript = result["text"]
   for wrong, correct in replacements.items():
       transcript = transcript.replace(wrong, correct)
   ```

3. Use cloud API for higher accuracy (lesson 2)

---

### Issue 4: High latency spikes

**Cause**: Model reloading or GPU context switching

**Solutions**:
1. Keep model loaded (don't reload for each transcription):
   ```python
   # GOOD: Load once
   model = WhisperModel("small")
   for audio in audio_files:
       model.transcribe(audio)

   # BAD: Reload every time
   for audio in audio_files:
       model = WhisperModel("small")  # Slow!
       model.transcribe(audio)
   ```

2. Warm up model with dummy audio:
   ```python
   import numpy as np

   # Create 1-second silent audio
   dummy_audio = np.zeros(16000, dtype=np.float32)
   model.transcribe(dummy_audio)  # Warmup

   # Now real transcriptions will be faster
   ```

3. Pin to GPU:
   ```python
   import torch
   torch.cuda.set_device(0)  # Lock to GPU 0
   ```

---

## Performance Optimization

### Optimization 1: Batch Processing

Process multiple audio files at once:

```python
from faster_whisper import WhisperModel
import concurrent.futures

model = WhisperModel("small", device="cuda", compute_type="float16")

def transcribe(audio_path):
    segments, _ = model.transcribe(audio_path, beam_size=1)
    return " ".join([s.text for s in segments])

# Process 10 files in parallel (CPU-bound, benefits from threading)
audio_files = [f"cmd_{i}.wav" for i in range(10)]

with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
    results = list(executor.map(transcribe, audio_files))
```

---

### Optimization 2: Streaming Inference

For ultra-low latency, process audio in overlapping windows:

```python
import numpy as np

def streaming_transcribe(audio_stream, window_seconds=3, overlap_seconds=1):
    """
    Transcribe audio stream with overlapping windows

    Args:
        audio_stream: Iterator of audio chunks (numpy arrays)
        window_seconds: Size of transcription window
        overlap_seconds: Overlap between windows
    """
    window_size = 16000 * window_seconds
    overlap_size = 16000 * overlap_seconds

    buffer = np.array([], dtype=np.float32)

    for chunk in audio_stream:
        buffer = np.concatenate([buffer, chunk])

        # Process when buffer is full
        if len(buffer) >= window_size:
            # Transcribe window
            segments, _ = model.transcribe(buffer[:window_size])
            text = " ".join([s.text for s in segments])
            yield text

            # Slide window (keep overlap)
            buffer = buffer[window_size - overlap_size:]
```

**Latency**: 3-second windows with 1-second overlap = ~2 seconds average latency

---

## Summary

In this lesson, you learned to:

- ✅ Install and run OpenAI Whisper models locally for offline inference
- ✅ Optimize Whisper with faster-whisper (4x speedup) and quantization (int8)
- ✅ Build complete STT pipeline with VAD, ring buffers, and error handling
- ✅ Integrate local Whisper with ROS 2 for real-time robotics
- ✅ Compare cloud API vs. local deployment trade-offs
- ✅ Benchmark performance across model sizes and hardware

**Key Takeaways**:
- **Model selection**: `small` is best balance for robotics (RTF ~10x on GPU)
- **Acceleration**: faster-whisper + int8 quantization → 4x speedup
- **VAD**: Silero VAD skips 90% of silent audio (saves compute)
- **Latency**: Local GPU inference: 100-500ms vs. Cloud API: 500-2000ms
- **Trade-off**: Cloud = highest accuracy, Local = lowest latency + offline

---

## Additional Resources

### Official Documentation
- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [faster-whisper GitHub](https://github.com/guillaumekln/faster-whisper)
- [Silero VAD](https://github.com/snakers4/silero-vad)

### Research Papers
- [Robust Speech Recognition via Large-Scale Weak Supervision (Whisper)](https://arxiv.org/abs/2212.04356)
- [CTranslate2: Fast Inference Engine](https://github.com/OpenNMT/CTranslate2)

### Performance Benchmarks
- [Whisper Model Benchmarks](https://github.com/openai/whisper#available-models-and-languages)
- [faster-whisper Benchmarks](https://github.com/guillaumekln/faster-whisper#benchmark)

### Video Tutorials
- [Local Whisper Setup Guide](https://www.youtube.com/watch?v=_example)
- [Optimizing Whisper for Edge Devices](https://www.youtube.com/watch?v=_example)

---

## Next Lesson

In **Lesson 4: Command Parsing**, you'll learn to:
- Extract intent and entities from voice transcripts
- Implement slot filling for robotics commands
- Map natural language to ROS 2 actions
- Handle ambiguous commands with clarification

Continue to [Command Parsing →](./command-parsing.md)
