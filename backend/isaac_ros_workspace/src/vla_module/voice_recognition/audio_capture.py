#!/usr/bin/env python3
"""
Audio capture module for the Vision-Language-Action (VLA) system.
Handles microphone input and audio preprocessing for voice command recognition.
"""

import rclpy
from rclpy.node import Node
import pyaudio
import numpy as np
import threading
import queue
import time
from sensor_msgs.msg import AudioData
from std_msgs.msg import String


class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')

        # Audio parameters
        self.rate = self.declare_parameter('audio_rate', 16000).value
        self.channels = self.declare_parameter('audio_channels', 1).value
        self.chunk_size = self.declare_parameter('chunk_size', 1024).value
        self.noise_threshold = self.declare_parameter('noise_threshold', 0.01).value
        self.silence_duration = self.declare_parameter('silence_duration', 1.0).value  # seconds

        # Publishers
        self.audio_pub = self.create_publisher(AudioData, '/audio_input', 10)

        # Audio stream variables
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.is_listening = False
        self.audio_queue = queue.Queue()

        # Audio preprocessing variables
        self.silence_counter = 0
        self.silence_threshold_samples = int(self.silence_duration * self.rate / self.chunk_size)

        # Start audio capture in a separate thread
        self.capture_thread = threading.Thread(target=self._capture_audio, daemon=True)

        # Timer to publish audio data periodically
        self.timer = self.create_timer(0.1, self._publish_audio_data)

        self.get_logger().info('Audio Capture Node initialized')

    def start_capture(self):
        """Start audio capture from microphone"""
        try:
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.rate,
                input=True,
                frames_per_buffer=self.chunk_size
            )
            self.is_listening = True
            self.capture_thread.start()
            self.get_logger().info('Audio capture started')
        except Exception as e:
            self.get_logger().error(f'Failed to start audio capture: {e}')

    def stop_capture(self):
        """Stop audio capture"""
        self.is_listening = False
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()
        self.get_logger().info('Audio capture stopped')

    def _capture_audio(self):
        """Internal method to capture audio in a separate thread"""
        while self.is_listening:
            try:
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)

                # Apply basic noise filtering
                filtered_data = self._apply_noise_filter(data)

                # Add audio data to queue for processing
                self.audio_queue.put(filtered_data)
            except Exception as e:
                self.get_logger().error(f'Error capturing audio: {e}')
                break

    def _apply_noise_filter(self, audio_data):
        """Apply basic noise filtering to audio data"""
        # Convert bytes to numpy array for processing
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # Apply basic noise reduction (simple spectral gating approach)
        # Calculate RMS amplitude
        rms = np.sqrt(np.mean(audio_array ** 2))

        # If the signal is below the noise threshold, reduce its amplitude
        if rms < self.noise_threshold:
            # Apply soft noise gate
            audio_array = audio_array * 0.1  # Reduce amplitude by 90%

        # Convert back to int16 format
        filtered_audio = (audio_array * 32767).astype(np.int16)

        # Convert back to bytes
        return filtered_audio.tobytes()

    def _publish_audio_data(self):
        """Publish audio data to ROS topic"""
        # Get all available audio data from queue
        audio_data = b''
        while not self.audio_queue.empty():
            chunk = self.audio_queue.get_nowait()
            audio_data += chunk

        if audio_data:
            # Detect voice activity before publishing
            if self.detect_voice_activity(audio_data):
                # Reset silence counter when voice activity is detected
                self.silence_counter = 0

                # Create and publish AudioData message
                msg = AudioData()
                msg.data = audio_data
                self.audio_pub.publish(msg)
            else:
                # Increment silence counter
                self.silence_counter += 1

                # Only publish silence data periodically to reduce network load
                if self.silence_counter % 10 == 0:  # Publish every 10th silence check
                    msg = AudioData()
                    msg.data = audio_data
                    self.audio_pub.publish(msg)

    def detect_voice_activity(self, audio_data):
        """Detect if there's voice activity in the audio data"""
        # Convert bytes to numpy array for processing
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # Calculate RMS (Root Mean Square) to measure volume
        rms = np.sqrt(np.mean(audio_array ** 2))

        # Return True if volume is above threshold (indicating voice activity)
        return rms > self.noise_threshold


def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()

    try:
        node.start_capture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.stop_capture()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()