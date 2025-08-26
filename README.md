# Line Maze Solver Robot 🤖

An intelligent Arduino-based line-following robot capable of autonomously navigating and solving complex mazes using advanced algorithms and path optimization techniques.

## 🚀 Features

### Core Capabilities
- **Autonomous Line Following**: PID-controlled line following with 8 IR sensors
- **Maze Solving**: Implements both left-hand and right-hand rule algorithms
- **Path Optimization**: Advanced LSRB (Left-Straight-Right-Back) path shortening algorithm
- **Dynamic Sensor Calibration**: Automatic sensor threshold adjustment for different lighting conditions
- **Dead-end Detection**: Intelligent U-turn execution when encountering dead ends
- **Goal Detection**: Automatic detection when maze is solved (black area detection)

### Advanced Features
- **Two-Phase Operation**: 
  1. **Scanning Phase**: Robot explores maze and records path
  2. **Solving Phase**: Robot follows optimized shortest path to goal
- **Path Storage**: Stores complete path in memory with optimization
- **Intersection Detection**: Handles T-intersections, cross-intersections, and goal areas
- **Anti-oscillation Protection**: Timing-based turn protection to prevent erratic behavior

## 🛠️ Hardware Components

| Component | Specification | Purpose |
|-----------|---------------|---------|
| **Microcontroller** | Arduino Nano | Main processing unit |
| **Motors** | 2x DC Geared Motors | Robot locomotion |
| **Motor Driver** | L298N | Motor control and power management |
| **Sensors** | 8x IR Sensors (A0-A7) | Line detection and navigation |
| **Input** | 4x Push Buttons | User interface and mode selection |
| **Output** | 2x LEDs | Status indication |
| **Power** | Battery Pack | Portable power supply |

## 🔧 Pin Configuration

```cpp
// Motor Control
#define ENA_PIN 5     // Right motor enable
#define ENB_PIN 6     // Left motor enable
#define IN1_PIN 3     // Right motor direction 1
#define IN2_PIN 2     // Right motor direction 2
#define IN3_PIN 7     // Left motor direction 1
#define IN4_PIN 4     // Left motor direction 2

// User Interface
#define BUTTON1_PIN 11  // Dynamic Calibration
#define BUTTON2_PIN 10  // Right-hand Rule Scan
#define BUTTON3_PIN 9   // Left-hand Rule Scan
#define BUTTON4_PIN 8   // Solve Maze

// Status LEDs
#define white_blue_led 12  // Activity indicator
#define RED_LED 13         // Status/Goal indicator

// Sensors: A0-A7 (8 IR sensors)
```

## 🎮 Operation Modes

### Button Functions
1. **Button 1**: Dynamic sensor calibration
2. **Button 2**: Right-hand rule maze scanning
3. **Button 3**: Left-hand rule maze scanning  
4. **Button 4**: Solve maze using optimized path

### Usage Instructions
1. **Setup**: Place robot at maze entrance
2. **Calibrate**: Press Button 1 for sensor calibration
3. **Scan**: Press Button 2 or 3 to explore maze and record path
4. **Solve**: Press Button 4 to execute optimized solution

## 🧠 Algorithms Implemented

### 1. PID Line Following
- **Proportional Control**: Immediate error correction
- **Derivative Control**: Smooth turning and stability
- **Integral Control**: Eliminates steady-state error
- **8-Sensor Weighted Position**: Precise line center calculation

### 2. Maze Solving Strategies
- **Left-Hand Rule**: Always prefer left turns when available
- **Right-Hand Rule**: Always prefer right turns when available
- **Dead-end Handling**: Automatic U-turn and backtracking

### 3. Path Optimization (LSRB Rules)
The robot implements 9 optimization patterns to shorten recorded paths:

```cpp
// Path Shortening Rules
LBR → B    // Left-Back-Right becomes Back
RBL → B    // Right-Back-Left becomes Back  
SBL → R    // Straight-Back-Left becomes Right
LBL → S    // Left-Back-Left becomes Straight
RBR → S    // Right-Back-Right becomes Straight
SBR → L    // Straight-Back-Right becomes Left
LBS → R    // Left-Back-Straight becomes Right
RBS → L    // Right-Back-Straight becomes Left
SBS → B    // Straight-Back-Straight becomes Back
```

### 4. Intersection Detection
- **T-Intersections**: Detects left/right path availability
- **Cross-Intersections**: Handles 4-way intersections
- **Goal Detection**: Identifies maze completion (black area)
- **Straight Path Continuation**: Maintains forward motion when possible

## 📊 Technical Specifications

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Base Speed** | 65 PWM | Default motor speed |
| **PID Gains** | Kp=60, Ki=0.2, Kd=1000 | Tuned for smooth operation |
| **Sensor Range** | 8 sensors | Full width coverage |
| **Turn Detection** | Edge sensors | Left/right path detection |
| **Calibration Time** | 5 seconds | Dynamic threshold setting |
| **Path Storage** | 100 moves | Maximum recordable path length |

## 🔬 Advanced Features

### Startup Protection
- 500ms initialization delay
- Prevents false dead-end detection at start
- Ensures stable sensor readings

### Turn Timing Protection
- Minimum 90ms between consecutive turns
- Prevents oscillation and erratic behavior
- Maintains smooth navigation

### Dynamic Braking
- Multi-stage motor stopping
- Precise positioning at intersections
- Reduced overshoot and improved accuracy

## 📈 Performance Optimizations

1. **Speed Optimization**: Optimized motor speeds for different maneuvers
2. **Memory Efficiency**: Efficient path storage and manipulation
3. **Sensor Reliability**: Dynamic calibration adapts to lighting conditions
4. **Path Efficiency**: LSRB algorithm reduces maze solving time by up to 60%

## 🚧 Project Status

✅ **Completed Features:**
- Line following with PID control
- Maze scanning (both left and right hand rules)
- Path recording and storage
- Path optimization algorithm
- Intersection and goal detection
- Complete maze solving workflow

🔄 **Potential Enhancements:**
- EEPROM integration for persistent path storage
- Wireless communication for remote monitoring
- LCD display for real-time status
- Speed optimization for competitive racing

## 🎯 Applications

- **Educational**: Robotics and AI learning platform
- **Competitive**: Maze solving competitions
- **Research**: Path planning algorithm testing
- **Industrial**: Automated guided vehicle (AGV) prototype

## 🤝 Contributing

Feel free to contribute to this project by:
- Reporting bugs or issues
- Suggesting new features
- Optimizing existing algorithms
- Adding documentation

## 📝 License

This project is open-source and available for educational and research purposes.

---

**Developed by:** Sricharan Panugothu  
**Project Type:** Arduino-based Autonomous Robot  
**Domain:** Robotics, Embedded Systems, AI Pathfinding
