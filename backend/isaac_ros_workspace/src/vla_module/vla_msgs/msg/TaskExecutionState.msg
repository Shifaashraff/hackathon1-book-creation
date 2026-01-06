# Represents the current state of task execution
string id                           # Unique identifier for the execution state
string cognitive_plan_id            # Reference to the plan being executed
string current_step                 # ID of the currently executing step
string[] completed_steps           # IDs of completed steps
string[] failed_steps              # IDs of failed steps
float32 progress                   # Overall progress (0.0 to 1.0)
string status                       # Current execution status (IDLE, EXECUTING, PAUSED, COMPLETED, FAILED, CANCELLED)
string error_message               # Error details if status is FAILED
string feedback                     # Human-readable feedback about execution
builtin_interfaces/Time start_time  # When execution started
builtin_interfaces/Time current_time # Current timestamp