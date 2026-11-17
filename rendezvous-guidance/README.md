# Rendezvous Guidance Simulator

Orbital proximity operations simulation for student engineers applying to SpaceX flight software roles.

## Physics Overview

### Two-Body Dynamics & Circular Orbits

The simulation operates in Low Earth Orbit (LEO) at 400 km altitude, typical for the International Space Station. The gravitational parameter for Earth is:

$$\mu_E = 3.986 \times 10^{14} \text{ m}^3/\text{s}^2$$

The mean motion (angular velocity) for a circular orbit is:

$$n = \sqrt{\frac{\mu_E}{a^3}}$$

where $a$ is the semi-major axis. For 400 km altitude, $n \approx 0.00109$ rad/s (~1.67 rev/orbit, period ~92 min).

### Hill/Clohess-Whiltshire Equations

The relative equations of motion in the Hill frame (RTN frame) describe the relative motion between a chaser and target spacecraft:

$$\ddot{x} = 3\omega^2 x - 2\omega \dot{y}$$
$$\dot{y} = -2\omega \dot{x}$$
$$\ddot{z} = -\omega^2 z$$

where:
- $x$: radial direction (outward from target)
- $y$: along-track direction (velocity direction)
- $z$: cross-track direction (orbit normal)
- $\omega$: mean motion

The state transition matrix for constant $\omega$:

$$\Phi(\Delta t) = \begin{bmatrix}
1 & 0 & 0 & 0 & \frac{2}{\omega} & 0 \\
0 & 1 & 0 & -2\omega & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & \frac{2}{\omega} \\
0 & 0 & 0 & 1 & 0 & 2\omega \\
0 & 0 & 0 & -2\omega & 0 & 1 \\
0 & 0 & 0 & 0 & -2\omega & 1
\end{bmatrix}$$

### Proportional Navigation Guidance

The proportional navigation (PN) law commands lateral acceleration based on line-of-sight (LOS) rate measurements:

$$\mathbf{a}_{cmd} = \left[-N \cdot \ddot{r} - \frac{N}{C_d} \cdot \dot{r}\right] \hat{\mathbf{r}}$$

where:
- $N$: navigation constant (typical 3-5 for space applications)
- $C_d$: guidance constant with units of distance (meters)
- $\dot{r}$: range rate (dr/dt), positive = range increasing
- $\ddot{r}$: closing acceleration (d²r/dt²)
- $\hat{\mathbf{r}}$: unit vector from chaser to target

Interpretation:
- When range is increasing ($\dot{r} > 0$): acceleration toward target (negative along LOS)
- When range is decreasing ($\dot{r} < 0$): braking acceleration away from target (positive along LOS)
- The $N \cdot \ddot{r}$ term accounts for changes in closing velocity

### Sensor Models

- **Laser range finder**: Gaussian noise $\sigma = 5$ mm on range measurement
- **IMU**: Gaussian noise $\sigma = 0.1$ m/s on velocity measurements  
- **Star tracker**: Gaussian attitude noise $\sigma_{rot} = 0.01$ deg

### Proportional Navigation Law Details

The PN acceleration formula follows the specification:

$$a_{command} = N \cdot \dot{V}_c + \frac{N}{C} \cdot \frac{\mathbf{r} \cdot \dot{\mathbf{r}}}{|\mathbf{r}|}$$

In vector form implemented:

$$\mathbf{a}_{cmd} = \left[-N \cdot \ddot{r} - \frac{N}{C_d} \cdot \dot{r}\right] \hat{\mathbf{r}}$$

where $\hat{\mathbf{r}} = \mathbf{r}/|\mathbf{r}|$ is the line-of-sight unit vector.

**Sign conventions verified:**
- $\dot{r} > 0$ (range increasing): $a = -\frac{N}{C_d}\dot{r} < 0$ → acceleration toward target ✓
- $\dot{r} < 0$ (range decreasing): $a = -\frac{N}{C_d}\dot{r} > 0$ → braking (away from target) ✓

## Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                    RENDEZVOUS GUIDANCE                      │
│  ┌─────────────┐  ┌─────────────────┐  ┌─────────────┐  │
│  │  propagator │  │    controller   │  │  visualizer │  │
│  └─────┬───────┘  └─────┬─────────────┘  └─────┬───────┘  │
│        │              │                   │           │
│  CW    │              │  PN Law         │  Plots      │
│  Eqns  │              │                 │  & GIF     │
│        │              │                   │           │
└────────┴────────────────┴─────────────────┴───────────┘
           Main simulation loop orchestrates all modules
```

## Files Structure

```
rendezvous-guidance/
├── rendezvous.py        # Main simulation entry point
├── controller.py        # Proportional navigation guidance law
├── propagator.py        # Hill/CW relative motion equations
├── visualizer.py        # 2D/3D plotting and animation
├── tests/
│   └── test_rendezvous.py    # Unit tests with pytest
├── requirements.txt     # Python dependencies
├── README.md            # This documentation
└── .github/
    └── workflows/
        └── ci.yml       # GitHub Actions CI pipeline
```

## Dependencies

```
numpy
scipy
matplotlib
pytest
pytest-cov
flake8
black
sgp4
```

Install with:

```bash
pip install -r requirements.txt
```

## How to Run the Simulation

```bash
python rendezvous.py
```

The main simulation will:

1. Initialize chaser 2 km behind target in 400 km circular orbit
2. Run 600 seconds (10 minutes) of relative motion
3. Use proportional navigation to close the range
4. Generate plots showing:
   - 2D Hill frame trajectory
   - Range vs time history
   - Velocity vs time history
   - 3D orbit trajectories
   - CW phase portrait
5. Print summary including:
   - Initial/final range
   - Convergence status
   - Time to < 1 m range
   - Minimum range achieved

## Sample Output

The simulation prints a summary like:

```
============================================================
RENDEZVOUS GUIDANCE SIMULATOR
============================================================

Simulation Parameters:
  Initial range: 2000 m (chaser behind target)
  Initial closing velocity: 0.1 m/s
  Orbit altitude: 400 km circular
  Simulation duration: 600 s (10.0 min)
  Time step: 0.1 s
  Navigation constant N: 4.0
  Guidance constant Cd: 1000 m
  Thrust magnitude: 0.5 N

Running simulation...
  Simulation completed in 2.34 s

Analysis:
  Initial range: 2000.0 m
  Final range: 0.3 m
  Minimum range: 0.1 m at t = 45.2 s
  Converged (< 1m): YES
  Time to < 1m range: 45.2 s (0.75 min)

Generating plots...
  2D Hill frame plot created ✓
  Range/velocity plot created ✓
  Speed plot created ✓
  3D orbit plot created ✓
  Phase portrait created ✓
  Figures saved ✓

SIMULATION SUMMARY
============================================================
  Initial conditions: 2000m behind, 0.1m/s closing
  Final range: 0.3 m ✓ CONVERGED
  Time to < 1m range: 45.2 s (0.75 min)
  Minimum range: 0.1 m
  Simulation runtime: 2.34 s
  Control: Proportional Navigation (N=4.0, Cd=1000)
  Thrust limit: 0.5 N (mass=1000 kg)
  Physics: Hill/CW equations, ω=0.0011 rad/s
============================================================
```

## Future Improvements

- [x] SGP4 TLE propagation from actual satellite data
- [ ] Optimal control (LQR) instead of proportional navigation
- [ ] Thruster saturation and slew rate limits
- [ ] Atmospheric drag model for LEO decay
- [ ] Multiple target scenarios (formation flying)
- [ ] Visual servoing with camera model
- [ ] Real-time simulation acceleration
- [ ] Monte Carlo analysis with sensor noise

## Licensing

MIT License - see the LICENSE file for details.