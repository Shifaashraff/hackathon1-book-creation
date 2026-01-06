# Represents a single action step in a cognitive plan
string id                           # Unique identifier for the action step
string action_type                  # Type of action (NAVIGATION, MANIPULATION, PERCEPTION, etc.)
string[] parameters                # Parameters for the action
int32 priority                      # Priority level for execution (lower number = higher priority)
string[] dependencies              # IDs of actions that must complete first
duration timeout                    # Maximum time allowed for this step
string[] success_criteria          # Conditions that define successful completion