# Voice-to-Action Exercises

## Overview
This document contains hands-on exercises for the Voice-to-Action chapter, focusing on OpenAI Whisper integration for voice commands.

## Exercise 1: Basic Voice Command Recognition

### Objective
Successfully recognize and process simple voice commands using the Whisper API.

### Steps
1. Launch the VLA system:
   ```bash
   ros2 launch vla_module vla_system.launch.py
   ```

2. Issue a simple command:
   - Speak clearly: "Move forward"
   - Observe the console output for recognition
   - Check the `/vla/voice_command` topic for the recognized command

3. Verify the command was processed:
   ```bash
   ros2 topic echo /vla/voice_command
   ```

### Expected Results
- Command should be recognized with confidence > 0.7
- Voice command message should appear in the topic
- Transcript should match "Move forward"

### Troubleshooting
- If recognition fails, check your OpenAI API key
- Ensure microphone is working and positioned correctly
- Verify network connectivity to OpenAI

## Exercise 2: Voice Activity Detection

### Objective
Test the voice activity detection system to optimize processing.

### Steps
1. Monitor audio input:
   ```bash
   ros2 topic echo /audio_input
   ```

2. Test with and without speaking:
   - Observe the topic during silence (should have reduced activity)
   - Speak and observe increased message frequency
   - Note the noise threshold parameters

3. Adjust noise threshold:
   - In a new terminal, change the parameter:
   ```bash
   ros2 param set /audio_capture_node noise_threshold 0.02
   ```

4. Test recognition at different threshold levels

### Expected Results
- Fewer messages during silence
- More messages during speech
- Recognition still works with higher thresholds

## Exercise 3: Command Complexity Testing

### Objective
Test recognition of increasingly complex voice commands.

### Steps
1. Test simple commands:
   - "Turn left"
   - "Move forward"
   - "Stop"

2. Test compound commands:
   - "Move forward and turn left"
   - "Go to the kitchen and pick up the red cup"

3. Test commands with numbers:
   - "Move forward 2 meters"
   - "Turn right 90 degrees"

4. Monitor the cognitive planning output:
   ```bash
   ros2 topic echo /vla/cognitive_plan
   ```

### Expected Results
- Simple commands should have high confidence (>0.8)
- Complex commands may have lower confidence but still be recognized
- Cognitive plans should reflect the complexity of the commands

## Exercise 4: Audio Quality Optimization

### Objective
Optimize audio quality and recognition performance.

### Steps
1. Test in different acoustic environments:
   - Quiet room
   - Room with moderate background noise
   - Room with high background noise

2. Adjust noise threshold for each environment:
   - Quiet: 0.005
   - Moderate: 0.01
   - High: 0.05

3. Measure recognition accuracy in each environment

4. Test different microphone positions:
   - 5cm from speaker
   - 10cm from speaker
   - 20cm from speaker

### Expected Results
- Higher accuracy in quieter environments
- Appropriate threshold settings for each environment
- Best results at 5-10cm distance

## Exercise 5: Error Handling and Edge Cases

### Objective
Test the system's response to various error conditions.

### Steps
1. Test with unclear speech:
   - Mumble or speak softly
   - Observe confidence thresholds
   - Check that low-confidence commands are rejected

2. Test with background noise:
   - Play music while giving commands
   - Observe the system's ability to filter noise
   - Adjust parameters as needed

3. Test with non-commands:
   - Say random phrases
   - Verify the system doesn't respond inappropriately
   - Check that only valid commands are processed

4. Test API failures:
   - Temporarily disconnect from the internet
   - Observe error handling
   - Reconnect and verify recovery

### Expected Results
- Low-confidence commands should be filtered out
- System should handle API errors gracefully
- Recovery should be automatic when connection is restored

## Exercise 6: Performance Monitoring

### Objective
Monitor system performance during voice command processing.

### Steps
1. Monitor CPU usage:
   ```bash
   htop
   ```

2. Monitor audio processing:
   ```bash
   ros2 topic hz /audio_input
   ```

3. Monitor command processing rate:
   ```bash
   ros2 topic hz /vla/voice_command
   ```

4. Record performance metrics:
   - Average response time
   - CPU usage during processing
   - Network usage
   - Recognition accuracy

### Expected Results
- CPU usage below 20% during normal operation
- Response time under 2 seconds
- Recognition accuracy above 85%

## Exercise 7: Integration Testing

### Objective
Test the complete voice-to-action pipeline.

### Steps
1. Launch the full system:
   ```bash
   ros2 launch vla_module vla_system.launch.py
   ```

2. Issue a complex command:
   - "Go to the kitchen and pick up the red cup"
   - Observe the complete pipeline:
     - Audio capture
     - Voice recognition
     - Cognitive planning
     - Task execution

3. Monitor all relevant topics:
   ```bash
   # In separate terminals:
   ros2 topic echo /audio_input
   ros2 topic echo /vla/voice_command
   ros2 topic echo /vla/cognitive_plan
   ros2 topic echo /vla/task_execution_state
   ```

4. Verify the complete flow from voice input to robot action

### Expected Results
- Complete pipeline should function end-to-end
- All topics should show appropriate messages
- Robot simulation should respond appropriately

## Assessment Criteria

### Success Metrics
- Voice command recognition accuracy: >85%
- Response time: \<2 seconds
- System stability: No crashes during testing
- Appropriate error handling: System recovers from errors

### Deliverables
For each exercise, document:
1. Steps taken
2. Results observed
3. Any issues encountered
4. Parameters adjusted
5. Performance metrics

## Advanced Challenges

### Challenge 1: Multi-language Support
- Configure the system for different languages
- Test recognition accuracy across languages
- Document any language-specific issues

### Challenge 2: Custom Voice Commands
- Define and test custom voice commands
- Extend the intent recognition system
- Create new command patterns

### Challenge 3: Performance Optimization
- Optimize audio processing for minimal latency
- Reduce network usage
- Improve recognition accuracy