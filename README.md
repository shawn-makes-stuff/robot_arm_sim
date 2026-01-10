# Robot Arm Simulator

A browser-based 3D robot arm simulator with terminal control interface. Built with Three.js for visualization and featuring inverse kinematics for position-based control.

![Robot Arm Simulator](https://img.shields.io/badge/Three.js-black?style=flat&logo=three.js) ![JavaScript](https://img.shields.io/badge/JavaScript-F7DF1E?style=flat&logo=javascript&logoColor=black)

## Features

- **5 DOF Articulated Arm**: Base rotation, shoulder, elbow, wrist bend, and wrist roll
- **Inverse Kinematics**: Move to XYZ coordinates with automatic joint angle calculation
- **Terminal Interface**: Command-line style control with tab completion
- **Animated Movements**: Smooth eased transitions between positions
- **Gripper Control**: Open/close gripper with percentage-based control
- **Save/Load Positions**: Store and recall custom arm configurations
- **Multiple View Presets**: Top, side, front, and free orbit camera views
- **Real-time Feedback**: Live joint angles and end-effector position display

## Quick Start

1. Clone the repository
2. Open `index.html` in a web browser
3. Type `help` in the terminal for available commands

No build process required - runs directly in the browser.

## Commands

### Joint Control
| Command | Alias | Description |
|---------|-------|-------------|
| `base <angle>` | `b` | Rotate base (-180° to 180°) |
| `shoulder <angle>` | `sh` | Tilt shoulder (-90° to 90°) |
| `elbow <angle>` | `e` | Bend elbow (-135° to 135°) |
| `wrist <angle>` | `w` | Bend wrist (-90° to 90°) |
| `rotate <angle>` | `r` | Roll wrist (-180° to 180°) |
| `move <b> <sh> <e> <w>` | - | Set all joints at once |

Use `+/-` prefix for relative adjustments: `b +10`, `e -15`

### Position Control (Inverse Kinematics)
| Command | Alias | Description |
|---------|-------|-------------|
| `goto <x> <y> <z>` | `g` | Move gripper to XYZ position (cm) |

Coordinates: X = left/right, Y = forward/back, Z = height

### Gripper
| Command | Description |
|---------|-------------|
| `grip <0-100>` | Set gripper openness (0=closed, 100=open) |
| `open` | Fully open gripper |
| `close` | Fully close gripper |

### Presets
| Command | Alias | Description |
|---------|-------|-------------|
| `preset <name>` | `p` | Load a preset position |
| `home` | - | Return to home stance |

Available presets: `home`, `upright`, `rest`, `reach`, `up`, `down`, `left`, `right`

### Save/Load
| Command | Description |
|---------|-------------|
| `save <name>` | Save current position |
| `load <name>` | Load saved position |
| `positions` | List saved positions |

### Utility
| Command | Description |
|---------|-------------|
| `status` | Show current arm state |
| `limits` | Show joint limits |
| `speed <ms>` | Set animation duration (100-5000ms) |
| `stop` | Stop current movement |
| `clear` | Clear terminal output |

## Coordinate System

The simulator uses a robotics-style coordinate system:
- **X**: Left (-) / Right (+)
- **Y**: Back (-) / Forward (+)
- **Z**: Down (floor=0) / Up (+)

The grid shows coordinates in centimeters with 100cm major divisions.

## Technical Details

- **Arm Segments**: Shoulder 50cm, Elbow 40cm, Wrist 15cm, Gripper 25cm
- **IK Solution**: Law of cosines with gripper-down constraint
- **Animation**: Cubic ease-in-out with configurable duration
- **Rendering**: Three.js with OrbitControls for camera manipulation

## Browser Support

Works in modern browsers with WebGL support (Chrome, Firefox, Safari, Edge).

## License

MIT
