# Audio Processing Guide

## Overview
This guide covers the audio processing pipeline in the Vision-Language-Action (VLA) system, including capture, preprocessing, and voice activity detection.

## Audio Pipeline Architecture

The audio processing pipeline consists of three main components:

1. **Audio Capture**: Captures audio from the microphone
2. **Preprocessing**: Filters and prepares audio for recognition
3. **Voice Activity Detection**: Detects speech segments to optimize processing

## Audio Capture

### Configuration
The audio capture node handles microphone input with the following configuration options:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `audio_rate` | 16000 | Sample rate in Hz |
| `audio_channels` | 1 | Number of audio channels (1=mono, 2=stereo) |
| `chunk_size` | 1024 | Size of audio chunks in samples |
| `noise_threshold` | 0.01 | Threshold for noise detection |
| `silence_duration` | 1.0 | Duration in seconds to consider silence |

### Audio Format
- Format: 16-bit integer (Int16)
- Sample Rate: 16000 Hz (configurable)
- Channels: Mono (configurable)
- Recommended: Use high-quality microphones for best results

## Audio Preprocessing

### Noise Filtering
The system implements basic noise filtering to improve recognition accuracy:

- **RMS-based filtering**: Reduces amplitude of low-level signals
- **Threshold-based**: Applies noise gate to signals below threshold
- **Adaptive**: Adjusts based on environmental noise levels

### Voice Activity Detection (VAD)
Voice Activity Detection helps optimize processing by:

- Reducing unnecessary API calls to Whisper
- Lowering network usage
- Improving response time
- Reducing computational load

The VAD system works by:
1. Calculating the RMS (Root Mean Square) of audio chunks
2. Comparing against a configurable threshold
3. Only processing chunks that exceed the threshold
4. Implementing a silence counter to detect when speech stops

## Performance Optimization

### Real-time Processing
To ensure real-time performance:

- Keep chunk size small (512-2048 samples)
- Process audio in separate threads
- Use efficient algorithms for real-time constraints
- Monitor CPU usage during processing

### Network Optimization
- Buffer audio data to reduce network calls
- Only send voice segments to Whisper API
- Implement silence detection to avoid sending silence
- Consider local processing for latency-critical applications

## Configuration Tuning

### Noise Threshold
Adjust the `noise_threshold` parameter based on your environment:

- **Quiet environment**: 0.005
- **Normal environment**: 0.01
- **Noisy environment**: 0.05

### Sample Rate
Choose the appropriate sample rate:

- **16000 Hz**: Standard for speech recognition
- **8000 Hz**: Lower quality, less processing
- **44100 Hz**: Higher quality, more processing (not recommended for real-time)

## Troubleshooting

### Common Issues

#### Audio Not Detected
- **Problem**: No audio input being captured
- **Solution**:
  - Check microphone permissions
  - Verify microphone is selected as input device
  - Test audio input with other applications

#### High CPU Usage
- **Problem**: Audio processing consuming too much CPU
- **Solution**:
  - Increase chunk size to reduce processing frequency
  - Optimize preprocessing algorithms
  - Consider using hardware acceleration

#### False Voice Detection
- **Problem**: System responding to non-speech sounds
- **Solution**:
  - Increase noise threshold
  - Fine-tune VAD parameters
  - Use more sophisticated VAD algorithms

#### Low Recognition Accuracy
- **Problem**: Whisper not recognizing speech clearly
- **Solution**:
  - Reduce noise threshold to capture more speech
  - Improve audio quality (microphone position, etc.)
  - Use noise reduction algorithms

## Advanced Topics

### Custom VAD Algorithms
For advanced implementations, consider:
- Spectral analysis for more accurate VAD
- Machine learning-based VAD models
- Integration with ROS 2 audio processing frameworks

### Multi-microphone Support
The system can be extended to support:
- Multiple microphones for better audio quality
- Beamforming for noise reduction
- Audio source localization

## Integration with ROS 2

### Audio Data Message
The system uses `sensor_msgs/AudioData` for audio transmission:
- Contains raw audio data in bytes
- Includes header with timestamp
- Published to `/audio_input` topic

### Audio Processing Pipeline
1. `audio_capture_node` publishes to `/audio_input`
2. `whisper_interface_node` subscribes to `/audio_input`
3. Processed commands published to `/vla/voice_command`

## Quality Metrics

### Performance Indicators
- **Latency**: Time from speech to command recognition
- **CPU Usage**: Processing overhead during audio capture
- **Network Usage**: Data sent to Whisper API
- **Recognition Accuracy**: Percentage of correctly recognized commands

### Monitoring
Monitor these topics for performance:
- `/audio_input` - Audio data flow
- `/vla/voice_command` - Recognition results
- System logs for error rates

## Best Practices

### Audio Quality
- Position microphone appropriately (5-10cm from speaker)
- Minimize background noise
- Use directional microphones when possible
- Test in the actual deployment environment

### Resource Management
- Monitor CPU and memory usage
- Implement proper cleanup of audio resources
- Handle audio device failures gracefully
- Consider power consumption on mobile robots

## Next Steps

- Review [Whisper Setup Guide](./whisper-setup.md) for API integration
- Complete the [Voice-to-Action Exercises](./exercises.md) for hands-on practice
- Explore [Cognitive Planning](../cognitive-planning/) for command processing