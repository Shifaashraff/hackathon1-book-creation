# Represents a voice command captured from user input
string id                    # Unique identifier for the command
builtin_interfaces/Time timestamp  # When the command was received
string transcript           # Transcribed text from speech
float32 confidence          # Confidence score from speech recognition (0.0-1.0)
string intent               # Parsed intent from the command
string[] parameters       # Parsed parameters from the command
string status               # Status of command processing (RECEIVED, PROCESSING, COMPLETED, FAILED)