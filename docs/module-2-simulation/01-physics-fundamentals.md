---
sidebar_position: 1
---

# Physics Simulation Fundamentals

Physics engines are the heart of robot simulation, numerically solving equations of motion to predict how objects move, collide, and interact. Understanding these fundamentals helps you configure simulators appropriately and recognize when sim-to-real transfer might fail.

## Newton's Laws in Discrete Time

Physical systems evolve continuously, but computers work in discrete timesteps. Physics engines approximate continuous dynamics through numerical integration.

### The Simulation Loop

At each timestep Δt (e.g., 0.001 seconds for 1000 Hz):

1. **Compute forces**: Gravity, contact forces, applied torques
2. **Update accelerations**: F = ma (Newton's second law)
3. **Integrate velocities**: v(t+Δt) = v(t) + a·Δt
4. **Integrate positions**: x(t+Δt) = x(t) + v·Δt
5. **Resolve constraints**: Enforce joint limits, prevent interpenetration

**Trade-Off**: Smaller Δt improves accuracy but increases computation time.

## Rigid Body Dynamics

Most simulators assume **rigid bodies**—objects that don't deform. Each body has:
- **Position and orientation** (pose in 3D space)
- **Linear and angular velocity**
- **Mass** and **inertia tensor** (resistance to rotation)

### Inertia Tensors

A robot link's resistance to rotation depends on mass distribution. The **inertia tensor** is a 3×3 matrix:

```
I = [ Ixx  Ixy  Ixz ]
    [ Ixy  Iyy  Iyz ]
    [ Ixz  Iyz  Izz ]
```

For a uniform box (width w, depth d, height h, mass m):
- Ixx = (1/12) · m · (d² + h²)
- Iyy = (1/12) · m · (w² + h²)
- Izz = (1/12) · m · (w² + d²)

**Incorrect inertias** cause unrealistic motion (e.g., a robot falls too slowly or spins erratically).

## Contact Dynamics: The Hard Problem

When two objects collide, determining contact forces is computationally expensive.

### Collision Detection

**Phases:**
1. **Broad phase**: Quickly eliminate distant object pairs (bounding boxes)
2. **Narrow phase**: Precisely detect contact points (mesh intersections)

**Collision Shapes:**
- **Primitives** (spheres, boxes, cylinders): Fast, accurate
- **Convex hulls**: Moderate complexity, good approximation
- **Triangle meshes**: Slow, detailed (use only for static objects)

**Best Practice**: Use simple collision shapes (convex decomposition of complex meshes).

### Contact Solvers

Once contact is detected, the solver computes forces preventing interpenetration.

**Penalty Method:**
- Apply spring-damper forces proportional to penetration depth
- Simple but can be unstable (objects vibrate or sink through surfaces)

**Constraint-Based (Impulse) Method:**
- Solve for impulses satisfying non-penetration constraints
- More stable, used in most modern engines (ODE, Bullet, PhysX)

## Friction Models

Friction resists relative motion between surfaces. **Coulomb friction** models static and kinetic friction:

- **Static friction** (μs): Prevents motion until force exceeds μs · N (normal force)
- **Kinetic friction** (μk): Opposes motion, typically μk < μs

**Example**: A robot foot on concrete might have μs = 0.9, μk = 0.7.

**Limitations**: Real-world friction is complex (temperature-dependent, surface roughness). Simulators use simplified models.

## Joint Constraints

Joints restrict relative motion between links. Implementing this requires:

### Positional Constraints

**Revolute joint** (hinge): Child can rotate around one axis relative to parent.

```
Constraint: distance(anchor_parent, anchor_child) = 0
```

Solvers apply forces to maintain this invariant.

### Limits and Damping

Joints have:
- **Position limits**: min/max angles (e.g., elbow: -2.0 to 0.5 rad)
- **Velocity limits**: max angular speed (prevents unrealistic motion)
- **Damping**: Dissipates energy (models friction in gears)

**Soft Limits**: Instead of hard stops, apply increasing resistance near limits (more realistic).

## Numerical Integration Methods

### Euler Integration (First-Order)

Simple but inaccurate:
```
v_new = v_old + a · Δt
x_new = x_old + v_old · Δt
```

**Problem**: Energy accumulation—objects gain speed over time (non-physical).

### Runge-Kutta (RK4)

Higher-order method evaluating derivatives at multiple points:
- More accurate for same Δt
- 4× computational cost per step

### Verlet Integration

Position-based, commonly used in games:
```
x_new = 2·x_current - x_previous + a · Δt²
```

**Advantage**: Naturally conserves energy (orbits don't spiral outward).

## Physics Engine Comparison

| Engine | ROS Integration | Strengths | Weaknesses |
|--------|----------------|-----------|------------|
| **ODE** | Gazebo Classic | Fast, stable | Less accurate contacts |
| **Bullet** | Gazebo, PyBullet | Good all-rounder | Moderate speed |
| **PhysX** | Isaac Sim, Unity | GPU-accelerated, accurate | Proprietary (NVIDIA) |
| **MuJoCo** | `dm_control`, RL | Excellent for control, differentiable | Complex setup |
| **DART** | Gazebo option | Precise contacts, articulated bodies | Slower |

## Simulation Accuracy vs. Speed

**Real-Time Requirement**: For interactive use, simulation must run at ≥1.0× real-time.

**Accuracy Factors:**
- Timestep size (smaller = more accurate)
- Solver iterations (more = better constraint satisfaction)
- Collision mesh complexity

**Example Settings** (Gazebo):
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
  <real_time_factor>1.0</real_time_factor>
  <ode>
    <solver>
      <type>quick</type>  <!-- Fast but less accurate -->
      <iters>50</iters>   <!-- 50 solver iterations -->
    </solver>
  </ode>
</physics>
```

## Domain Randomization for Sim-to-Real

Exact physical modeling is impossible. **Domain randomization** creates robust policies by varying parameters:

**Randomize:**
- Friction coefficients (±30%)
- Masses (±10%)
- Joint damping (±20%)
- Time delays (sensor latency 0-50ms)

**Result**: Policies learn to handle uncertainty, generalizing to real-world discrepancies.

## Common Pitfalls

### Pitfall 1: Ignoring Inertias

Setting all inertias to identity matrix causes:
- Unrealistic rotations (spinning like a top)
- Instability in contact (falls through floors)

**Solution**: Use CAD software to compute accurate inertias or use heuristics for simple shapes.

### Pitfall 2: Timestep Too Large

Δt = 0.01s (100 Hz) can miss fast collisions.

**Solution**: Use Δt ≤ 0.001s for stable simulation of legged robots.

### Pitfall 3: Over-Constraining Joints

Redundant constraints (e.g., fixed joint + zero velocity limit) cause solver failures.

**Solution**: Use minimal constraints; let physics handle the rest.

## Conclusion

Physics simulation approximates reality through numerical methods, with trade-offs between accuracy and speed. Understanding rigid body dynamics, contact solvers, and integration methods helps you configure simulators effectively and anticipate where sim-to-real transfer may struggle. In the next chapter, we'll apply this knowledge in Gazebo, simulating a humanoid robot.

## References

- Mirtich, B. (1996). "Impulse-based dynamic simulation of rigid body systems." PhD thesis, University of California, Berkeley.
- Todorov, E., Erez, T., & Tassa, Y. (2012). "MuJoCo: A physics engine for model-based control." *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 5026-5033.
- Coumans, E., & Bai, Y. (2016). "PyBullet, a Python module for physics simulation for games, robotics and machine learning." Retrieved from http://pybullet.org
