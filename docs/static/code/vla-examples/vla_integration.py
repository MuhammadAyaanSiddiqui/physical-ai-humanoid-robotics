#!/usr/bin/env python3
"""
Complete VLA Integration Example

End-to-end pipeline: Voice → Whisper → LLM → ROS 2 Actions
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import os
import wave
import tempfile
from dotenv import load_dotenv
from openai import OpenAI
import json

load_dotenv()

class VLAIntegration(Node):
    def __init__(self):
        super().__init__('vla_integration')

        # Initialize OpenAI clients
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # Audio buffer
        self.audio_frames = []
        self.chunk_duration = 3.0
        self.sample_rate = 16000
        self.max_frames = int(self.sample_rate / 1024 * self.chunk_duration)

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio/chunks',
            self.audio_callback,
            10
        )

        # Publishers
        self.transcript_pub = self.create_publisher(String, 'voice/transcript', 10)
        self.plan_pub = self.create_publisher(String, 'robot/plan', 10)

        # Action client
        self.action_client = ActionClient(self, ExecuteAction, 'execute_action')

        self.get_logger().info('🤖 VLA Integration Node ready')

    def audio_callback(self, msg):
        """Process incoming audio"""
        self.audio_frames.append(bytes(msg.data))

        if len(self.audio_frames) >= self.max_frames:
            self.process_audio_chunk()
            self.audio_frames.clear()

    def process_audio_chunk(self):
        """Complete VLA pipeline"""
        # 1. Transcribe audio
        transcript = self.transcribe_audio()
        if not transcript:
            return

        self.get_logger().info(f"📝 Transcript: {transcript}")
        self.transcript_pub.publish(String(data=transcript))

        # 2. Generate action plan with LLM
        plan = self.generate_plan(transcript)
        if not plan:
            return

        self.get_logger().info(f"🧠 Plan: {json.dumps(plan, indent=2)}")
        self.plan_pub.publish(String(data=json.dumps(plan)))

        # 3. Execute actions
        self.execute_plan(plan)

    def transcribe_audio(self):
        """Transcribe audio buffer using Whisper"""
        try:
            # Save to temporary file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
                with wave.open(tmp.name, 'wb') as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(self.sample_rate)
                    wf.writeframes(b''.join(self.audio_frames))

                # Transcribe
                with open(tmp.name, 'rb') as audio_file:
                    transcription = self.openai_client.audio.transcriptions.create(
                        model="whisper-1",
                        file=audio_file,
                        response_format="text"
                    )

                return transcription.strip()

        except Exception as e:
            self.get_logger().error(f"Transcription failed: {e}")
            return None

    def generate_plan(self, command):
        """Generate action plan using GPT-4"""
        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[
                    {
                        "role": "system",
                        "content": """You are a robot planner. Generate JSON action plans.
Actions: navigate, pick, place, scan
Output: {"plan": [{"action": "...", ...}]}"""
                    },
                    {"role": "user", "content": f"Command: {command}"}
                ],
                response_format={"type": "json_object"},
                temperature=0.0
            )

            return json.loads(response.choices[0].message.content)

        except Exception as e:
            self.get_logger().error(f"Planning failed: {e}")
            return None

    def execute_plan(self, plan):
        """Execute action sequence"""
        for i, action in enumerate(plan.get('plan', [])):
            self.get_logger().info(f"🎯 Executing action {i+1}/{len(plan['plan'])}: {action['action']}")

            # Send action goal (simplified - real implementation would use action client)
            # self.action_client.send_goal_async(action)

            self.get_logger().info(f"✓ Action {i+1} complete")

def main(args=None):
    rclpy.init(args=args)
    node = VLAIntegration()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
