# Isaac Sim Test Scenes for Humanoid Navigation

This directory contains test scenes for validating Isaac Sim humanoid navigation capabilities.

## Scene Descriptions

### humanoid_navigation_test.usd
A comprehensive test scene that includes:

- **Ground Plane**: Standard environment floor
- **Navigation Obstacles**: Static obstacles for path planning testing
- **Narrow Passage**: Tight corridor for navigation challenge
- **Stairs**: Stepped obstacles for testing humanoid step climbing
- **Ramp**: Sloped surface for testing slope navigation
- **Small Obstacles**: Low obstacles for stepping over tests
- **Camera Setup**: Multiple cameras for perception testing

## Usage

To load a test scene in Isaac Sim:

1. Open Isaac Sim
2. Go to File → Open Stage
3. Select one of the USD files in this directory
4. Configure your humanoid robot in the scene

## Scene Features

### Navigation Challenges
- Static obstacles of various sizes and positions
- Narrow passages requiring precise navigation
- Stairs and ramps for testing locomotion capabilities
- Small obstacles to test step-over abilities

### Perception Testing
- Multiple camera angles for synthetic data generation
- Various lighting conditions for robust perception
- Different obstacle types for detection testing

### Bipedal Locomotion
- Stairs for testing step climbing
- Ramps for testing slope navigation
- Various ground textures for stability testing

## Test Scenarios

### Basic Navigation
1. Load the scene
2. Place the humanoid robot at the starting position
3. Set navigation goals around obstacles
4. Verify successful path planning and execution

### Obstacle Avoidance
1. Test navigation with dynamic obstacle avoidance
2. Verify that the robot maintains balance during avoidance
3. Check that obstacles are properly detected and avoided

### Stair Climbing
1. Navigate the humanoid robot up and down stairs
2. Verify stable footstep planning
3. Check balance maintenance during stair climbing

### Perception Integration
1. Use the camera setup for synthetic data generation
2. Test perception pipeline with various obstacles
3. Validate detection accuracy in the scene

## Scene Customization

You can modify the test scenes by:
- Adding new obstacles
- Changing lighting conditions
- Adjusting camera positions
- Modifying terrain features

## Troubleshooting

- Ensure all paths in the USD files are correct
- Verify that the humanoid robot model is compatible with the scene
- Check that Isaac Sim has sufficient resources to render the scene