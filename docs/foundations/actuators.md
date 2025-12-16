---
sidebar_position: 2
---

# Actuators: Translating Digital Commands into Physical Motion

While sensors provide the input to Physical AI systems, **actuators** are the output—the mechanisms that convert electrical signals into mechanical motion. For humanoid robots, actuators enable locomotion, manipulation, and interaction with the environment. Understanding actuator types, their characteristics, and control methods is essential for designing capable and efficient robotic systems.

## The Role of Actuators in Robotics

Actuators are the muscles of a robot. They receive control signals (voltages, currents, or digital commands) and produce motion—linear displacement, rotational movement, or force application. The quality of a robot's performance is fundamentally limited by its actuators' capabilities:

- **Speed**: How quickly can the actuator respond and move?
- **Precision**: How accurately can it achieve target positions?
- **Force/Torque**: How much load can it handle?
- **Energy efficiency**: How much power does it consume relative to output?
- **Durability**: How long can it operate before failure?

## Electric Motors: The Dominant Actuator

The vast majority of modern humanoid robots use **electric motors** as their primary actuators. Electric motors convert electrical energy into rotational motion through electromagnetic principles.

### DC Motors

**Direct current (DC) motors** are simple, reliable, and widely available. A DC motor consists of a rotor (armature) within a magnetic field created by permanent magnets or electromagnets. When current flows through the rotor windings, electromagnetic forces cause rotation.

**Advantages**:
- Simple control via voltage or PWM (pulse-width modulation)
- Continuous rotation with controllable speed
- Widely available and cost-effective

**Limitations**:
- Brushed DC motors require maintenance (brushes wear out)
- Limited torque at low speeds without gearing
- Noise and electromagnetic interference from commutation

**Example**: A low-cost humanoid research platform might use brushed DC motors for non-critical joints (neck rotation, wrist yaw) where occasional maintenance is acceptable.

### Brushless DC Motors (BLDC)

**Brushless DC motors** eliminate mechanical brushes by using electronic commutation. Sensors (Hall effect or encoders) detect rotor position, and a controller energizes coil windings in sequence to maintain rotation.

**Advantages**:
- Higher efficiency (less energy lost to friction and heat)
- Longer lifespan (no brush wear)
- Higher power-to-weight ratio
- Better heat dissipation

**Limitations**:
- Require more complex controllers (motor drivers with position feedback)
- Higher cost than brushed motors

**Example**: High-performance humanoid robots like Boston Dynamics' Atlas use brushless motors for joint actuation, enabling efficient, high-torque operation.

### Servo Motors

A **servo motor** is a DC motor (brushed or brushless) integrated with a position sensor (encoder or potentiometer) and a control circuit. The controller continuously adjusts motor output to maintain a commanded position or velocity.

**Key Features**:
- Closed-loop control for precise positioning
- Built-in feedback mechanism
- Standard interfaces (PWM control signal for hobby servos, industrial protocols for advanced servos)

**Types**:
- **Hobby servos**: Inexpensive, limited rotation (typically 180°), used in small robots and prototypes
- **Industrial servos**: High torque, continuous rotation, absolute encoders, used in professional humanoid platforms

**Example**: A humanoid hand uses servo motors in each finger joint, enabling precise grip force and finger positioning for object manipulation.

## Linear Actuators

While rotational motion is common, some applications require **linear motion**—extending and retracting along a straight line. Linear actuators include:

- **Lead screws**: Rotating a threaded shaft moves a nut linearly (high force, slow speed)
- **Belt drives**: A motor drives a belt, translating rotational to linear motion (fast, moderate force)
- **Linear motors**: Direct electromagnetic linear force without mechanical conversion (high precision, expensive)

**Example**: A telescoping robot arm segment uses a linear actuator to extend reach, allowing the robot to adjust its effective arm length dynamically.

## Hydraulic and Pneumatic Actuators

### Hydraulic Actuators

**Hydraulic systems** use pressurized fluid (oil) to generate force. A pump pressurizes the fluid, which drives pistons or hydraulic motors.

**Advantages**:
- Extremely high force and power density
- Smooth, continuous motion
- Natural compliance (fluid compressibility absorbs shocks)

**Limitations**:
- Complex infrastructure (pumps, reservoirs, hoses)
- Potential for leaks and contamination
- Heavy and energy-intensive
- Noisy operation

**Example**: Boston Dynamics' Atlas uses hydraulic actuators for explosive, dynamic movements (parkour, backflips) that electric motors struggle to achieve.

### Pneumatic Actuators

**Pneumatic systems** use compressed air instead of liquid. They are lighter than hydraulics but produce lower forces.

**Advantages**:
- Lightweight and simple
- Naturally compliant (air compression acts as a spring)
- Safe (leaks are less problematic than hydraulic fluid)

**Limitations**:
- Lower force density than hydraulics
- Difficult to control precisely (air compressibility introduces nonlinearity)
- Require air compressors and pressure regulation

**Example**: Soft robotic grippers use pneumatic actuators to gently conform to object shapes, ideal for handling delicate items like food or biological samples.

## Motor Control Fundamentals

Actuators don't operate in isolation—they require **control systems** to achieve desired behaviors.

### Pulse-Width Modulation (PWM)

**PWM** controls motor speed by rapidly switching power on and off. The duty cycle (percentage of time "on") determines average voltage and thus speed. Most microcontrollers (Arduino, Raspberry Pi) provide PWM outputs for motor control.

### PID Control

**Proportional-Integral-Derivative (PID)** controllers are the workhorse of actuator control. A PID controller continuously calculates an error (difference between desired and actual position) and adjusts motor output:

- **Proportional (P)**: Correction proportional to current error
- **Integral (I)**: Correction based on accumulated past error (eliminates steady-state error)
- **Derivative (D)**: Correction based on rate of error change (dampens oscillation)

Tuning PID gains (Kp, Ki, Kd) is critical for stable, responsive control.

**Example**: A humanoid robot maintaining balance uses PID controllers on ankle joints, adjusting torque based on IMU measurements to keep the torso upright.

## Actuator Selection Considerations

Choosing the right actuator involves trade-offs:

| Requirement | Best Actuator Type |
|-------------|-------------------|
| High precision, moderate force | Electric servo motor with encoder |
| Extreme force, dynamic motion | Hydraulic actuator |
| Lightweight, compliant grasping | Pneumatic or soft actuator |
| Energy efficiency | Brushless DC motor with gearing |
| Low cost, simple control | Brushed DC motor |

## Gearing and Transmission

Motors typically operate at high speeds (thousands of RPM) but low torque. Humanoid robots require the opposite—high torque at low speeds for walking and lifting. **Gearboxes** trade speed for torque:

- **Spur gears**: Simple, efficient, but noisy
- **Planetary gears**: Compact, high torque, used in robotic joints
- **Harmonic drives**: Zero backlash, high reduction ratios, precise positioning

**Backlash** (play in gears) is problematic for precision tasks. High-quality gearboxes (harmonic drives, cycloidal drives) minimize backlash but increase cost.

## Energy Efficiency and Power Management

Actuators are the primary energy consumers in mobile robots. Efficiency strategies include:

- **Regenerative braking**: Capturing energy when motors decelerate, returning it to the battery
- **Variable impedance**: Adjusting joint stiffness dynamically to reduce energy waste
- **Motion planning**: Choosing trajectories that minimize acceleration and jerk

A humanoid robot walking for hours on battery power must optimize every motion for energy conservation.

## Safety Mechanisms

Actuators can exert dangerous forces. Safety features include:

- **Current limiting**: Detecting excessive motor current (indicating collision or jam) and stopping motion
- **Force-torque sensors**: Measuring joint forces and halting if thresholds are exceeded
- **Compliant actuators**: Series elastic actuators (SEAs) with springs that absorb impacts, reducing collision forces
- **Emergency stops**: Hardware kill switches that immediately cut power to all actuators

**Example**: A collaborative robot (cobot) working near humans uses compliant actuators and force sensing to stop immediately upon contact, preventing injury.

## Conclusion

Actuators are the physical manifestation of a robot's intelligence—the link between computation and action. Understanding motor types, control principles, and trade-offs in force, speed, precision, and efficiency is foundational to designing capable humanoid robots. As we move into practical ROS 2 modules, you'll see how software interfaces with these actuators, sending velocity commands, position targets, and force references to bring robotic systems to life.

In the next chapter, we explore the digital twin concept—how simulation environments model actuators and physics to enable safe, rapid development before real-world deployment.

## References

- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot Modeling and Control* (2nd ed.). Wiley.
- Siciliano, B., & Khatib, O. (Eds.). (2016). *Springer Handbook of Robotics* (2nd ed.). Springer.
- Pratt, G. A., & Williamson, M. M. (1995). "Series elastic actuators." *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 399-406.
