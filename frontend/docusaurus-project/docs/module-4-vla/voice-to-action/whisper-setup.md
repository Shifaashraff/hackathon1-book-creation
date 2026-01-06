# Whisper Setup Guide

## Overview
This guide explains how to set up the OpenAI Whisper API integration for voice command recognition in the Vision-Language-Action (VLA) system.

## Prerequisites

### OpenAI Account
- Create an account at [OpenAI](https://platform.openai.com/)
- Generate an API key from the OpenAI dashboard
- Ensure you have sufficient credits for API usage

### System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.8+
- NVIDIA GPU (recommended for Isaac Sim)

## Installation

### 1. Install Python Dependencies
```bash
pip3 install openai numpy scipy pyaudio
```

### 2. Set Up OpenAI API Key
```bash
export OPENAI_API_KEY="your-api-key-here"
```

For permanent setup, add the export command to your `~/.bashrc` file:
```bash
echo 'export OPENAI_API_KEY="your-api-key-here"' >> ~/.bashrc
source ~/.bashrc
```

### 3. Verify Installation
```bash
python3 -c "import openai; print(openai.__version__)"
```

## Configuration

### ROS 2 Parameters
The Whisper interface node accepts the following parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `openai_api_key` | `""` | OpenAI API key (will use environment variable if not set) |
| `whisper_model` | `"whisper-1"` | Whisper model to use |
| `language` | `"en"` | Language code for transcription |
| `confidence_threshold` | `0.7` | Minimum confidence for accepting transcription |

### Audio Configuration
- `audio_rate`: Sample rate for audio capture (default: 16000 Hz)
- `audio_channels`: Number of audio channels (default: 1 for mono)
- `chunk_size`: Audio buffer size (default: 1024 samples)
- `noise_threshold`: Threshold for noise detection (default: 0.01)

## Usage

### Launch the Whisper Interface
```bash
ros2 launch vla_module vla_system.launch.py
```

### Test Voice Commands
Once the system is running, you can issue voice commands to the humanoid robot:

1. Ensure the audio capture node is running
2. Speak a command like "Move forward" or "Turn left"
3. Observe the robot's response in the console and simulation

## Troubleshooting

### Common Issues

#### API Key Issues
- **Problem**: "AuthenticationError" or "Invalid API key"
- **Solution**: Verify your API key is correctly set in the environment

#### Audio Issues
- **Problem**: No audio input detected
- **Solution**: Check microphone permissions and audio device settings

#### High Latency
- **Problem**: Delay between command and response
- **Solution**: Check network connectivity and consider using a local Whisper model

#### Low Recognition Accuracy
- **Problem**: Commands not recognized accurately
- **Solution**:
  - Ensure quiet environment
  - Speak clearly and at consistent volume
  - Adjust noise threshold parameter

## Performance Optimization

### Network Optimization
- Use a fast, stable internet connection
- Consider using a local proxy if experiencing latency

### Audio Quality
- Use a high-quality microphone when possible
- Minimize background noise
- Position microphone appropriately for clear voice capture

## Advanced Configuration

### Custom Models
The system can be configured to use different Whisper models:
- `whisper-1`: Default model, good balance of speed and accuracy
- Future: Local models for offline processing

### Language Support
The system supports multiple languages through the `language` parameter:
- `en`: English
- `es`: Spanish
- `fr`: French
- `de`: German
- And more as supported by OpenAI

## Security Considerations

### API Key Security
- Never hardcode API keys in source code
- Use environment variables or secure parameter servers
- Rotate keys regularly
- Monitor API usage for unexpected activity

### Data Privacy
- Voice data is sent to OpenAI for processing
- Consider local processing alternatives for sensitive applications
- Follow organizational privacy policies

## Next Steps

- Learn about [Audio Processing](./audio-processing.md) for advanced audio handling
- Explore [Cognitive Planning](../cognitive-planning/) for command interpretation
- Complete the [Voice-to-Action Exercises](./exercises.md) to practice implementation