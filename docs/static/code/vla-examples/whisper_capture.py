#!/usr/bin/env python3
"""
Whisper Audio Capture and Transcription Example

Records audio from ReSpeaker microphone and transcribes using Whisper API.
"""

import pyaudio
import wave
import os
from dotenv import load_dotenv
from openai import OpenAI

# Configuration
RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 1
RESPEAKER_INDEX = 1
CHUNK_DURATION = 5  # seconds

load_dotenv()
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def record_audio(duration=5, filename="recording.wav"):
    """Record audio from microphone"""
    p = pyaudio.PyAudio()

    stream = p.open(
        rate=RESPEAKER_RATE,
        format=pyaudio.paInt16,
        channels=RESPEAKER_CHANNELS,
        input=True,
        input_device_index=RESPEAKER_INDEX,
        frames_per_buffer=1024
    )

    print(f"Recording for {duration} seconds...")

    frames = []
    for _ in range(0, int(RESPEAKER_RATE / 1024 * duration)):
        data = stream.read(1024)
        frames.append(data)

    stream.stop_stream()
    stream.close()
    p.terminate()

    # Save to file
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(RESPEAKER_CHANNELS)
        wf.setsampwidth(2)
        wf.setframerate(RESPEAKER_RATE)
        wf.writeframes(b''.join(frames))

    print(f"Saved to {filename}")
    return filename

def transcribe_audio(filename):
    """Transcribe audio using Whisper API"""
    with open(filename, "rb") as audio_file:
        transcription = client.audio.transcriptions.create(
            model="whisper-1",
            file=audio_file,
            response_format="text"
        )

    return transcription

def main():
    """Main execution loop"""
    print("Whisper Audio Capture")
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            input("Press Enter to start recording (or Ctrl+C to quit)...")

            # Record
            audio_file = record_audio(duration=CHUNK_DURATION)

            # Transcribe
            print("Transcribing...")
            transcript = transcribe_audio(audio_file)

            print(f"\nTranscript: {transcript}\n")

    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
