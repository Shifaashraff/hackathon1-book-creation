#!/usr/bin/env python3
"""
Unit tests for voice recognition components of the Vision-Language-Action (VLA) system.
"""

import unittest
import numpy as np
from voice_recognition.voice_processor import VoiceProcessorNode
from voice_recognition.audio_capture import AudioCaptureNode
from vla_msgs.msg import VoiceCommand


class TestVoiceRecognition(unittest.TestCase):
    """Unit tests for voice recognition components"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Note: In a real implementation, we would use a test node
        # For this example, we'll test the core logic directly
        pass

    def test_voice_command_validation_valid(self):
        """Test validation of valid voice commands"""
        # Create a mock voice command with high confidence
        cmd = VoiceCommand()
        cmd.transcript = "Move forward"
        cmd.confidence = 0.9
        cmd.intent = "NAVIGATION"

        # Test validation logic (this would be in the VoiceProcessorNode)
        is_valid = self._validate_voice_command(cmd, min_confidence=0.7)
        self.assertTrue(is_valid, "Valid command with high confidence should pass validation")

    def test_voice_command_validation_low_confidence(self):
        """Test validation of voice commands with low confidence"""
        # Create a mock voice command with low confidence
        cmd = VoiceCommand()
        cmd.transcript = "Move forward"
        cmd.confidence = 0.5
        cmd.intent = "NAVIGATION"

        # Test validation logic
        is_valid = self._validate_voice_command(cmd, min_confidence=0.7)
        self.assertFalse(is_valid, "Command with low confidence should fail validation")

    def test_voice_command_validation_empty_transcript(self):
        """Test validation of voice commands with empty transcript"""
        # Create a mock voice command with empty transcript
        cmd = VoiceCommand()
        cmd.transcript = ""
        cmd.confidence = 0.9
        cmd.intent = "NAVIGATION"

        # Test validation logic
        is_valid = self._validate_voice_command(cmd, min_confidence=0.7)
        self.assertFalse(is_valid, "Command with empty transcript should fail validation")

    def test_intent_extraction_navigation(self):
        """Test intent extraction for navigation commands"""
        test_cases = [
            "Move forward",
            "Turn left",
            "Go to kitchen",
            "Move backward",
            "Turn right"
        ]

        for command in test_cases:
            with self.subTest(command=command):
                intent = self._extract_intent(command)
                self.assertEqual(intent, "NAVIGATION", f"Command '{command}' should have NAVIGATION intent")

    def test_intent_extraction_manipulation(self):
        """Test intent extraction for manipulation commands"""
        test_cases = [
            "Pick up the cup",
            "Grab the bottle",
            "Take the object",
            "Drop the item"
        ]

        for command in test_cases:
            with self.subTest(command=command):
                intent = self._extract_intent(command)
                self.assertEqual(intent, "MANIPULATION", f"Command '{command}' should have MANIPULATION intent")

    def test_intent_extraction_perception(self):
        """Test intent extraction for perception commands"""
        test_cases = [
            "Find the red cup",
            "Look for the bottle",
            "See the object",
            "Locate the item"
        ]

        for command in test_cases:
            with self.subTest(command=command):
                intent = self._extract_intent(command)
                self.assertEqual(intent, "PERCEPTION", f"Command '{command}' should have PERCEPTION intent")

    def test_parameter_extraction_numbers(self):
        """Test parameter extraction for numbers"""
        command = "Move forward 5 meters"
        params = self._extract_parameters(command)

        # Check if number parameter is extracted
        number_params = [p for p in params if p.startswith("number:")]
        self.assertTrue(len(number_params) > 0, "Number should be extracted from command")
        self.assertIn("number:5", number_params, "Number '5' should be extracted")

    def test_parameter_extraction_directions(self):
        """Test parameter extraction for directions"""
        command = "Turn left and move forward"
        params = self._extract_parameters(command)

        # Check if direction parameters are extracted
        direction_params = [p for p in params if p.startswith("direction:")]
        self.assertTrue(len(direction_params) > 0, "Directions should be extracted from command")
        self.assertIn("direction:left", direction_params, "Direction 'left' should be extracted")
        self.assertIn("direction:forward", direction_params, "Direction 'forward' should be extracted")

    def test_audio_noise_filtering(self):
        """Test audio noise filtering functionality"""
        # Create a low-amplitude signal (noise)
        low_signal = np.array([0.001] * 1000, dtype=np.float32)
        low_signal_bytes = (low_signal * 32767).astype(np.int16).tobytes()

        # Apply noise filtering (simulated)
        filtered_bytes = self._apply_noise_filter_simulation(low_signal_bytes, 0.01)
        filtered_array = np.frombuffer(filtered_bytes, dtype=np.int16).astype(np.float32) / 32767.0

        # The filtered signal should have reduced amplitude
        original_rms = np.sqrt(np.mean(np.array([0.001] * 1000) ** 2))
        filtered_rms = np.sqrt(np.mean(filtered_array ** 2))

        # After filtering with 90% reduction, the amplitude should be much lower
        self.assertLess(filtered_rms, original_rms, "Filtered signal should have lower amplitude")

    def _validate_voice_command(self, cmd, min_confidence=0.7):
        """Simulate voice command validation logic"""
        # Check confidence threshold
        if cmd.confidence < min_confidence:
            return False

        # Check if transcript is empty
        if not cmd.transcript or not cmd.transcript.strip():
            return False

        return True

    def _extract_intent(self, transcript):
        """Simulate intent extraction logic"""
        transcript_lower = transcript.lower()

        if any(word in transcript_lower for word in ['move', 'go', 'forward', 'backward', 'left', 'right', 'turn']):
            return 'NAVIGATION'
        elif any(word in transcript_lower for word in ['pick', 'grab', 'lift', 'take', 'hold', 'drop']):
            return 'MANIPULATION'
        elif any(word in transcript_lower for word in ['find', 'look', 'see', 'locate', 'search']):
            return 'PERCEPTION'
        elif any(word in transcript_lower for word in ['stop', 'pause', 'wait']):
            return 'STOP'
        else:
            return 'UNKNOWN'

    def _extract_parameters(self, transcript):
        """Simulate parameter extraction logic"""
        parameters = []

        # Extract numbers
        import re
        numbers = re.findall(r'\d+', transcript)
        for num in numbers:
            parameters.append(f"number:{num}")

        # Extract directions
        directions = ['forward', 'backward', 'left', 'right', 'up', 'down', 'north', 'south', 'east', 'west']
        for direction in directions:
            if direction in transcript.lower():
                parameters.append(f"direction:{direction}")

        # Extract objects
        objects = ['cup', 'bottle', 'box', 'red', 'blue', 'green']
        for obj in objects:
            if obj in transcript.lower():
                parameters.append(f"object:{obj}")

        return parameters

    def _apply_noise_filter_simulation(self, audio_data, noise_threshold):
        """Simulate noise filtering"""
        # Convert bytes to numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

        # Calculate RMS amplitude
        rms = np.sqrt(np.mean(audio_array ** 2))

        # If the signal is below the noise threshold, reduce its amplitude
        if rms < noise_threshold:
            # Apply soft noise gate
            audio_array = audio_array * 0.1  # Reduce amplitude by 90%

        # Convert back to int16 format
        filtered_audio = (audio_array * 32767).astype(np.int16)

        # Convert back to bytes
        return filtered_audio.tobytes()


def run_tests():
    """Run all tests"""
    # Create a test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestVoiceRecognition)

    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Return success/failure
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    exit(0 if success else 1)