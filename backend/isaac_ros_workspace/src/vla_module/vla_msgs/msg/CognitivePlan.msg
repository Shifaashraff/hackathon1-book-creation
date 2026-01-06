# Represents a cognitive plan generated from a voice command
string id                           # Unique identifier for the plan
string voice_command_id             # Reference to the original voice command
builtin_interfaces/Time timestamp   # When the plan was created
ActionStep[] actions               # Sequence of actions to execute
string status                       # Current status (PLANNING, READY, EXECUTING, COMPLETED, FAILED)
duration estimated_duration         # Estimated time to complete the plan
string[] required_capabilities     # Robot capabilities needed for execution