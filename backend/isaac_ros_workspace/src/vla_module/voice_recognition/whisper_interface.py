#!/usr/bin/env python3
"""
Whisper interface module for the Vision-Language-Action (VLA) system.
Handles integration with OpenAI Whisper API for voice command recognition.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
from std_msgs.msg import String
from vla_msgs.msg import VoiceCommand
import openai
import tempfile
import wave
import numpy as np
import os


class WhisperInterfaceNode(Node):
    def __init__(self):
        super().__init__('whisper_interface_node')

        # Get OpenAI API key from environment or parameter
        api_key = self.declare_parameter('openai_api_key', '').value
        if not api_key:
            api_key = os.getenv('OPENAI_API_KEY', '')
        if not api_key:
            self.get_logger().error('OpenAI API key not provided. Set OPENAI_API_KEY environment variable.')

        # Initialize OpenAI client
        self.client = openai.OpenAI(api_key=api_key)

        # Subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )

        # Publishers
        self.voice_command_pub = self.create_publisher(VoiceCommand, '/vla/voice_command', 10)

        # Configuration parameters
        self.model = self.declare_parameter('whisper_model', 'whisper-1').value
        self.language = self.declare_parameter('language', 'en').value
        self.confidence_threshold = self.declare_parameter('confidence_threshold', 0.7).value

        self.get_logger().info('Whisper Interface Node initialized')

    def audio_callback(self, msg):
        """Process incoming audio data and convert to text using Whisper API"""
        try:
            # Convert audio data to WAV format for Whisper API
            wav_data = self._convert_audio_data_to_wav(msg.data)

            # Create temporary file for Whisper API
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_file.write(audio_data)
                temp_file_path = temp_file.name

            try:
                # Transcribe audio using Whisper API
                with open(temp_file_path, 'rb') as audio_file:
                    transcript = self.client.audio.transcriptions.create(
                        model="whisper-1",
                        file=audio_file,
                        language=self.language
                    )

                # Process the transcription result
                self._process_transcription(transcript, temp_file_path)

            finally:
                # Clean up temporary file
                if os.path.exists(temp_file_path):
                    os.remove(temp_file_path)

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def _convert_audio_data_to_wav(self, audio_data):
        """Convert raw audio data to WAV format suitable for Whisper API"""
        import struct

        # For this implementation, we'll create a basic WAV structure
        sample_rate = 16000  # Assuming 16kHz sample rate
        channels = 1         # Mono
        sample_width = 2     # 16-bit samples

        # Create WAV file with proper header
        wav_buffer = b''

        # WAV header (simplified)
        wav_buffer += b'RIFF'  # Chunk ID
        wav_buffer += (len(audio_data) + 36).to_bytes(4, byteorder='little')  # Chunk size
        wav_buffer += b'WAVE'  # Format
        wav_buffer += b'fmt '  # Subchunk1 ID
        wav_buffer += (16).to_bytes(4, byteorder='little')  # Subchunk1 size
        wav_buffer += (1).to_bytes(2, byteorder='little')   # Audio format (1 = PCM)
        wav_buffer += channels.to_bytes(2, byteorder='little')  # Num channels
        wav_buffer += sample_rate.to_bytes(4, byteorder='little')  # Sample rate
        wav_buffer += (sample_rate * channels * sample_width).to_bytes(4, byteorder='little')  # Byte rate
        wav_buffer += (channels * sample_width).to_bytes(2, byteorder='little')  # Block align
        wav_buffer += (sample_width * 8).to_bytes(2, byteorder='little')  # Bits per sample
        wav_buffer += b'data'  # Subchunk2 ID
        wav_buffer += len(audio_data).to_bytes(4, byteorder='little')  # Subchunk2 size
        wav_buffer += audio_data  # Audio data

        return wav_buffer

    def _process_transcription(self, transcript, audio_file_path):
        """Process the transcription result and publish VoiceCommand message"""
        try:
            text = transcript.text.strip()
            # Note: OpenAI Whisper API doesn't provide confidence scores directly
            # We'll use a default high confidence for successful transcriptions
            confidence = 0.95

            if text:
                # Create VoiceCommand message
                voice_cmd = VoiceCommand()
                voice_cmd.id = f"whisper_{self.get_clock().now().nanoseconds}"
                voice_cmd.timestamp = self.get_clock().now().to_msg()
                voice_cmd.transcript = text
                voice_cmd.confidence = confidence
                voice_cmd.status = "RECEIVED"  # Initial status

                # Basic intent parsing (simplified)
                voice_cmd.intent = self._parse_intent(text)

                # Extract parameters from the command
                voice_cmd.parameters = self._extract_parameters(text)

                # Publish the voice command
                self.voice_command_pub.publish(voice_cmd)

                self.get_logger().info(f'Voice command recognized: "{text}" (confidence: {confidence:.2f})')
            else:
                self.get_logger().info('No text in transcription result')
        except Exception as e:
            self.get_logger().error(f'Error processing transcription: {e}')

    def _parse_intent(self, text):
        """Simple intent parser to extract command intent from text"""
        text_lower = text.lower()

        # Simple keyword-based intent detection
        if any(word in text_lower for word in ['move', 'go', 'forward', 'backward', 'left', 'right', 'turn', 'navigate']):
            return 'NAVIGATION'
        elif any(word in text_lower for word in ['pick', 'grab', 'lift', 'take', 'hold', 'drop', 'place', 'grasp']):
            return 'MANIPULATION'
        elif any(word in text_lower for word in ['find', 'look', 'see', 'locate', 'search', 'detect', 'identify']):
            return 'PERCEPTION'
        elif any(word in text_lower for word in ['stop', 'pause', 'wait', 'halt']):
            return 'STOP'
        elif any(word in text_lower for word in ['hello', 'hi', 'hey', 'start', 'begin']):
            return 'GREETING'
        else:
            return 'UNKNOWN'

    def _extract_parameters(self, text):
        """Extract parameters from command text"""
        parameters = []

        # Extract objects
        objects = ['cup', 'bottle', 'box', 'ball', 'red', 'blue', 'green', 'cup', 'table', 'kitchen', 'living room', 'bedroom']
        for obj in objects:
            if obj in text.lower():
                parameters.append(f"object:{obj}")

        # Extract directions
        directions = ['forward', 'backward', 'left', 'right', 'up', 'down', 'north', 'south', 'east', 'west']
        for direction in directions:
            if direction in text.lower():
                parameters.append(f"direction:{direction}")

        # Extract locations
        locations = ['kitchen', 'bedroom', 'living room', 'bathroom', 'office', 'dining room']
        for loc in locations:
            if loc in text.lower():
                parameters.append(f"location:{loc}")

        # Extract numbers/distance
        import re
        numbers = re.findall(r'\d+', text)
        for num in numbers:
            parameters.append(f"number:{num}")

        return parameters


def main(args=None):
    rclpy.init(args=args)
    node = WhisperInterfaceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()