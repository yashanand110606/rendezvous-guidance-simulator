"""
Rendezvous Guidance Simulator - Main Simulation Script

Orbital proximity operations simulation for student engineers applying to SpaceX
flight software roles.

Simulates chaser approaching target using Hill/Clohess-Whiltshire relative motion
equations and proportional navigation guidance.

Physics:
- Hill frame (RTN): x radial outward, y along-track, z cross-track
- CW equations: relative motion in near-circular orbit
- PN controller: proportional navigation for guidance
- Sensor models: Gaussian noise on range, velocity, attitude

Simulation:
- Duration: 600 seconds (10 minutes)
- Time step: 0.1 seconds
- Initial: chaser 2km behind target, 400km circular orbit
- Control: thrust 0.1-1.0 N
"""

import numpy as np
import sys
import time
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from typing import Tuple, Dict, Optional

# Physical constants
EARTH_RADIUS = 6371000.0  # m mean equatorial radius
MU_EARTH = 3.986004418e14  # m^3/s^2 gravitational parameter


def mean_motion(semi_major_axis: float) -> float:
    """Compute the mean motion (angular velocity) for a circular orbit."""
    return np.sqrt(MU_EARTH / semi_major_axis ** 3)


def clo_hill_whitstle_derivatives(t, state, omega):
    """Compute derivatives for Hill/CW equations of motion."""
    x, y, z, xdot, ydot, zdot = state

    # CW equations
    xddot = 3 * omega ** 2 * x - 2 * omega * ydot
    yddot = 2 * omega * xdot
    zddot = -omega ** 2 * z

    return np.array([xdot, ydot, zdot, xddot, yddot, zddot])


def propagate_hill_frame(state_chaser, state_target, dt, omega):
    """Propagate relative state in Hill frame using linearized CW equations."""
    # CW state transition matrix Phi(dt)
    Phi = np.array(
        [
            [1, 0, 0, 0, 2 / omega, 0],
            [0, 1, 0, -2 * omega, 0, 0],
            [0, 0, 1, 0, 0, 2 / omega],
            [0, 0, 0, 1, 0, 2 * omega],
            [0, 0, 0, -2 * omega, 0, 1],
            [0, 0, 0, 0, -2 * omega, 1],
        ]
    )

    # Propagate: state_new = Phi @ state_old
    state_new = Phi @ state_chaser

    return state_new


def run_simulation(
    initial_range: float = 2000.0,  # m - chaser behind target
    initial_closing_vel: float = 0.1,  # m/s
    orbit_altitude: float = 400e3,  # m - 400km
    simulation_time: float = 600.0,  # s - 10 min
    time_step: float = 0.1,  # s
    navigation_constant: float = 4.0,  # N - typical 3-5
    guidance_constant: float = 10.0,  # Cd - m (adjusted for control effectiveness)
    thrust_magnitude: float = 0.5,  # N - adjustable
    use_numerical_integration: bool = True,
) -> Dict[str, np.ndarray]:
    """
    Run the rendezvous guidance simulation.
    """
    print("="*60)
    print("RENDEZVOUS GUIDANCE SIMULATOR")
    print("="*60)
    print(f"\nSimulation Parameters:")
    print(f"  Initial range: {initial_range:.0f} m (chaser {'behind' if initial_range > 0 else 'ahead of'} target)")
    print(f"  Initial closing velocity: {initial_closing_vel:.1f} m/s")
    print(f"  Orbit altitude: {orbit_altitude/1000:.0f} km circular")
    print(f"  Simulation duration: {simulation_time:.0f} s ({simulation_time/60:.1f} min)")
    print(f"  Time step: {time_step:.1f} s")
    print(f"  Navigation constant N: {navigation_constant:.1f}")
    print(f"  Guidance constant Cd: {guidance_constant:.1f} m")
    print(f"  Thrust magnitude: {thrust_magnitude:.1f} N")
    print()

    # Compute mean motion for the orbit
    omega = mean_motion(orbit_altitude + EARTH_RADIUS)  # Earth radius + altitude
    print(f"  Mean motion omega: {omega:.6f} rad/s = {omega*60/np.pi:.1f} deg/min")

    # =========================================================================
    # Initial Conditions
    # =========================================================================
    # Hill frame: x radial outward, y along-track (velocity direction), z cross-track
    # Target at origin [0, 0, 0]
    # Chaser initially 2km behind in along-track (y direction), at same radial level

    # Initial relative state: [x, y, z, xdot, ydot, zdot]
    # y negative means chaser is "behind" (negative along-track direction)
    # ydot negative means chaser is moving toward target (closing)
    state_chaser_init = np.array([
        0.0,                     # x: radial position (0 = same radial level)
        -initial_range,          # y: along-track (negative = behind target)
        0.0,                     # z: cross-track
        0.0,                     # xdot: radial velocity
        -initial_closing_vel,    # ydot: along-track velocity (negative = closing)
        0.0                      # zdot: cross-track velocity
    ])

    state_target_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    print(f"\nInitial Conditions (Hill frame):")
    print(f"  Chaser position: x={state_chaser_init[0]:.0f}m, y={state_chaser_init[1]:.0f}m, z={state_chaser_init[2]:.0f}m")
    print(f"  Chaser velocity: xdot={state_chaser_init[3]:.4f}m/s, ydot={state_chaser_init[4]:.4f}m/s, zdot={state_chaser_init[5]:.4f}m/s")
    print(f"  Target at origin: [0, 0, 0]")
    print()

    # =========================================================================
    # Time Setup
    # =========================================================================
    n_steps = int(simulation_time / time_step) + 1
    times = np.linspace(0, simulation_time, n_steps)

    print(f"Time discretization: {n_steps} steps over {simulation_time:.0f}s ({time_step}s step)")
    print()

    # =========================================================================
    # Allocate Result Arrays
    # =========================================================================
    chaser_positions = np.zeros((n_steps, 3))
    chaser_states = np.zeros((n_steps, 6))  # Store full state history for plotting
    target_positions = np.zeros((n_steps, 3))
    ranges = np.zeros(n_steps)
    range_rates = np.zeros(n_steps)
    accelerations = np.zeros((n_steps, 3))

    # =========================================================================
    # Simulation Loop
    # =========================================================================
    print("Running simulation...")
    start_time = time.time()

    # Current state
    state_chaser = state_chaser_init.copy()
    state_target = state_target_init.copy()

    # For tracking
    min_range = float('inf')
    range_at_min = float('inf')

    # ======================================================================
    # Main integration loop
    # ======================================================================
    for i in range(n_steps):
        t = times[i]

        # --- Compute acceleration command using PN law ---
        # Get range vector from chaser to target
        # In Hill frame, target is at origin, chaser state gives relative position
        r_vec = np.array([state_chaser[0], state_chaser[1], state_chaser[2]])  # [x, y, z]
        r_norm = np.linalg.norm(r_vec)

        # Range rate: derivative of range
        # r_dot = (r Â· v_rel) / r, where relative velocity is from state
        # If chaser is at (x, y) and has velocities (xdot, ydot),
        # then r = sqrt(x^2 + y^2) and r_dot = (x*xdot + y*ydot) / r

        if r_norm > 1e-3:
            r_dot = (state_chaser[0] * state_chaser[3] + state_chaser[1] * state_chaser[4]) / r_norm
        else:
            r_dot = 0.0

        # Compute CW second derivatives
        derivs = clo_hill_whitstle_derivatives(t, state_chaser, omega)
        xddot = derivs[3]  # x'' (radial acceleration component)
        yddot = derivs[4]  # y'' (along-track acceleration component)

        # Effective closing acceleration projected onto LOS
        # r_ddot = (xdot^2 + x*xddot + ydot^2 + y*yddot)/r - r_dot^2
        x, y = state_chaser[0], state_chaser[1]
        xdot, ydot = state_chaser[3], state_chaser[4]

        if r_norm > 1e-3:
            r_ddot = (xdot**2 + x * xddot + ydot**2 + y * yddot) / r_norm - r_dot**2
        else:
            r_ddot = 0.0

        # Proportional navigation acceleration law:
        # a_cmd = [-N * r_ddot - (N/Cd) * r_dot] * r_hat
        # Where:
        # - r_ddot = closing acceleration (dÂ²r/dtÂ²)
        # - r_dot = range rate (dr/dt), positive = range increasing
        # - r_hat = unit vector from chaser to target
        # - N = navigation constant (typical 3-5)
        # - Cd = guidance constant (meters)

        # The formula drives acceleration toward target when range is increasing,
        # and braking when range is decreasing.
        if r_norm > 1e-3:
            r_hat = r_vec / r_norm
        else:
            r_hat = np.array([1.0, 0.0, 0.0])  # Default direction

        # PN acceleration scalar and vector
        scalar_part = -navigation_constant * r_ddot - (navigation_constant / guidance_constant) * r_dot
        acceleration_vec = scalar_part * r_hat

        # Apply thrust magnitude limit
        # Assuming chaser mass is 1000 kg (typical small satellite)
        chaser_mass = 1000.0  # kg
        max_accel = thrust_magnitude / chaser_mass  # m/s^2

        a_cmd_mag = np.linalg.norm(acceleration_vec)
        if a_cmd_mag > max_accel:
            # Scale down to maximum available
            acceleration_vec = acceleration_vec / a_cmd_mag * max_accel

        # Store acceleration
        accelerations[i] = acceleration_vec

        # Store chaser state for plotting
        chaser_states[i] = state_chaser.copy()

        # --- Propagate chaser state forward ---
        # Using Euler integration with control

        # Get derivatives from CW equations (without control)
        derivs = clo_hill_whitstle_derivatives(t, state_chaser, omega)

        # Add control input: acceleration affects the velocity derivatives
        # Map acceleration vector to Hill frame components:
        # acceleration_vec = [ax, ay, az] where:
        # - ax: radial direction (x-axis of Hill frame)
        # - ay: along-track direction (y-axis of Hill frame)
        # - az: cross-track direction (z-axis of Hill frame)
        ax_hill = acceleration_vec[0]  # radial component (along x)
        ay_hill = acceleration_vec[1]  # along-track component (along y)
        az_hill = acceleration_vec[2]  # cross-track component (along z)

        # Update state derivative: add control acceleration
        new_derivs = derivs.copy()
        new_derivs[3] += ax_hill  # xddot += radial acceleration
        new_derivs[4] += ay_hill  # yddot += along-track acceleration
        new_derivs[5] += az_hill  # zddot += cross-track acceleration

        # Euler step
        state_chaser = state_chaser + time_step * new_derivs

        # Also propagate target (stays at origin for reference target)
        # state_target stays [0,0,0,0,0,0]

        # --- Store results ---
        # Position (first 3 state components)
        chaser_positions[i] = state_chaser[:3]

        # Range and range rate
        r_current = np.linalg.norm(state_chaser[:3])
        ranges[i] = r_current

        if r_current > 1e-3:
            range_rates[i] = (state_chaser[0] * state_chaser[3] + state_chaser[1] * state_chaser[4]) / r_current
        else:
            range_rates[i] = 0.0

        # Track minimum range
        if r_current < min_range:
            min_range = r_current
            range_at_min = r_current

        # Optional: break early if converged
        if r_current < 1.0 and i > 10:  # Converged after initial transient
            pass

    elapsed_time = time.time() - start_time
    print(f"  Simulation completed in {elapsed_time:.2f}s\n")

    # =========================================================================
    # Post-processing and Analysis
    # =========================================================================
    print("Analysis:")
    print(f"  Initial range: {ranges[0]:.1f} m")
    print(f"  Final range:   {ranges[-1]:.1f} m")
    print(f"  Minimum range: {min_range:.1f} m")
    print(f"  Range at minimum: at t = {times[np.argmin(ranges)]:.1f} s")

    # Check convergence
    converged = ranges[-1] < 1.0  # < 1 meter final range
    time_to_zero = None
    for i in range(n_steps):
        if ranges[i] < 1.0:
            time_to_zero = times[i]
            break

    if time_to_zero is None:
        time_to_zero = simulation_time

    print(f"  Converged (< 1m): {'YES' if converged else 'NO'}")
    if time_to_zero is not None:
        print(f"  Time to < 1m range: {time_to_zero:.1f} s ({time_to_zero/60:.2f} min)")

    # =========================================================================
    # Generate Visualizations
    # =========================================================================
    print("\nGenerating plots...")

    # 2D Hill frame trajectory
    try:
        fig_2d = plot_2d_hill_frame(
            target_state=state_target_init,
            chaser_states=chaser_states,
            time=times,
            title=f"Rendezvous Approach: {initial_range}m initial range"
        )
        os.makedirs("plots", exist_ok=True)
        fig_2d.savefig(f"plots/rendezvous_2d_hill_{initial_range}m.png", dpi=150, bbox_inches='tight')
        plt.close(fig_2d)
        print("  2D plot saved âœ“")
    except Exception as e:
        print(f"  2D plot error: {e}")

    # Range and velocity history
    try:
        # Compute actual velocities from states
        vel_magnitudes = np.sqrt(chaser_states[:, 3]**2 + chaser_states[:, 4]**2 + chaser_states[:, 5]**2)

        fig, ax_range, ax_vel = plot_range_velocity_history(
            times, ranges, range_rates, vel_magnitudes,
            title="Range and Velocity History"
        )
        fig.savefig(f"plots/rendezvous_range_vel_{initial_range}m.png", dpi=150, bbox_inches='tight')
        plt.close(fig)
        print("  Range/vel plot saved âœ“")
    except Exception as e:
        print(f"  Range/vel plot error: {e}")

    # Speed history
    try:
        fig2, ax_speed = plot_velocity_history(
            times, vel_magnitudes, title="Chaser Speed History"
        )
        fig2.savefig(f"plots/rendezvous_speed_{initial_range}m.png", dpi=150, bbox_inches='tight')
        plt.close(fig2)
        print("  Speed plot saved âœ“")
    except Exception as e:
        print(f"  Speed plot error: {e}")

    # 3D orbits (sampled subset for clarity)
    try:
        n_points_3d = min(100, n_steps)
        chaser_3d = chaser_positions[::max(1, n_steps // n_points_3d)]
        target_3d = target_positions[::max(1, n_steps // n_points_3d)]

        # Simple 3D plot
        fig3 = plt.figure(figsize=(10, 9))
        ax3d = fig3.add_subplot(111, projection='3d')

        # Plot target trajectory (just origin since it's reference)
        ax3d.plot([0], [0], [0], 'yo', markersize=10, label='Target')

        # Plot chaser trajectory
        if np.any(chaser_3d):
            ax3d.plot(
                chaser_3d[:, 0],
                chaser_3d[:, 1],
                chaser_3d[:, 2],
                'b-', linewidth=2, alpha=0.8, label='Chaser trajectory'
            )

        # Mark initial and final positions
        if chaser_positions.shape[0] > 0:
            ax3d.plot(
                chaser_positions[0, 0], chaser_positions[0, 1], chaser_positions[0, 2],
                'go', markersize=8, label='Chaser start'
            )
            ax3d.plot(
                chaser_positions[-1, 0], chaser_positions[-1, 1], chaser_positions[-1, 2],
                'ro', markersize=8, label='Chaser end'
            )

        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Y (m)')
        ax3d.set_zlabel('Z (m)')
        ax3d.set_title('3D Rendezvous Trajectories')
        ax3d.grid(True, alpha=0.3)
        ax3d.legend()

        # Set equal aspect ratio approximation
        all_x = np.concatenate([np.array([0]), chaser_positions[:, 0]])
        all_y = np.concatenate([np.array([0]), chaser_positions[:, 1]])
        all_z = np.concatenate([np.array([0]), chaser_positions[:, 2]])

        if len(all_x) > 0:
            max_range_val = max(np.max(all_x) - np.min(all_x),
                                np.max(all_y) - np.min(all_y),
                                np.max(all_z) - np.min(all_z))
            mean_range = (np.max(all_x) + np.min(all_x) +
                          np.max(all_y) + np.min(all_y) +
                          np.max(all_z) + np.min(all_z)) / 6
            ax3d.set_xlim(mean_range - max_range_val/2, mean_range + max_range_val/2)
            ax3d.set_ylim(mean_range - max_range_val/2, mean_range + max_range_val/2)
            ax3d.set_zlim(mean_range - max_range_val/2, mean_range + max_range_val/2)

        fig3.savefig(f"plots/rendezvous_3d_{initial_range}m.png", dpi=150, bbox_inches='tight')
        plt.close(fig3)
        print("  3D plot saved âœ“")
    except Exception as e:
        print(f"  3D plot error: {e}")

    # CW phase portrait
    try:
        fig_phase, ax_phase = plt.subplots(figsize=(10, 8))
        ax_phase.plot(chaser_states[:, 0], chaser_states[:, 1], 'b-', alpha=0.7, linewidth=1)
        ax_phase.plot(chaser_states[0, 0], chaser_states[0, 1], 'go', markersize=8, label='Initial')
        ax_phase.plot(chaser_states[-1, 0], chaser_states[-1, 1], 'ro', markersize=8, label='Final')
        ax_phase.set_xlabel('Radial position x (m)')
        ax_phase.set_ylabel('Along-track position y (m)')
        ax_phase.set_title('CW Phase Portrait: x-y Relative Motion')
        ax_phase.grid(True, alpha=0.3)
        ax_phase.set_aspect('equal')
        ax_phase.legend()
        fig_phase.savefig(f"plots/rendezvous_phase_{initial_range}m.png", dpi=150, bbox_inches='tight')
        plt.close(fig_phase)
        print("  Phase portrait saved âœ“")
    except Exception as e:
        print(f"  Phase portrait error: {e}")

    print("\n" + "="*60)
    print("SIMULATION SUMMARY")
    print("="*60)
    print(f"  Initial conditions: {initial_range}m behind, {initial_closing_vel}m/s closing")
    print(f"  Final range: {ranges[-1]:.1f} m {'[CONVERGED]' if converged else '[NOT CONVERGED]'}")
    if time_to_zero is not None:
        print(f"  Time to < 1m: {time_to_zero:.1f} s")
    print(f"  Minimum range: {min_range:.1f} m")
    print(f"  Simulation runtime: {elapsed_time:.2f} s")
    print(f"  Control: Proportional Navigation (N={navigation_constant}, Cd={guidance_constant})")
    print(f"  Thrust limit: {thrust_magnitude} N (mass={chaser_mass} kg)")
    print(f"  Physics: Hill/CW equations, Ï‰={omega:.4f} rad/s")
    print("="*60)

    # Return results
    results = {
        'time': times,
        'chaser_positions': chaser_positions,
        'target_positions': target_positions,
        'ranges': ranges,
        'range_rates': range_rates,
        'accelerations': accelerations,
        'final_range': ranges[-1],
        'final_time': times[-1],
        'converged': converged,
        'time_to_zero': time_to_zero,
        'min_range': min_range,
        'omega': omega,
    }

    return results


def run_parameter_sweep():
    """Run a parameter sweep over key simulation parameters."""
    print("="*60)
    print("PARAMETER SWEEP")
    print("="*60)

    results_list = []

    # Sweep over initial ranges
    initial_ranges = [500, 1000, 2000, 3000]  # meters

    for init_range in initial_ranges:
        print(f"\n--- Initial range: {init_range} m ---")
        results = run_simulation(
            initial_range=init_range,
            initial_closing_vel=0.1,
            orbit_altitude=400e3,
            simulation_time=600.0,
            time_step=0.1,
            navigation_constant=4.0,
            guidance_constant=10.0,
            thrust_magnitude=0.5,
            use_numerical_integration=True,
        )
        results_list.append({
            'initial_range': init_range,
            'final_range': results['final_range'],
            'converged': results['converged'],
            'time_to_zero': results['time_to_zero'],
            'min_range': results['min_range'],
        })

    # Print summary table
    print("\n\n" + "="*60)
    print("PARAMETER SWEEP SUMMARY")
    print("="*60)
    print(f"{'Initial Range':>15} | {'Final Range':>12} | {'Converged':>10} | {'Time to 1m':>12}")
    print("-" * 60)
    for r in results_list:
        tc = r['time_to_zero'] if r['time_to_zero'] else 600.0
        print(f"{r['initial_range']:>15} | {r['final_range']:>12.1f} | {str(r['converged']):>10} | {tc:>12.1f}")

    return results_list


# Helper functions for visualizer (inline definitions)
def plot_2d_hill_frame(target_state, chaser_states, time, title="2D Rendezvous - Hill Frame", show_target=True, show_trajectory=True):
    """Plot 2D relative motion in Hill frame (x-y plane)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # Extract positions
    if chaser_states.ndim == 2 and chaser_states.shape[1] >= 3:
        chaser_x = chaser_states[:, 0]
        chaser_y = chaser_states[:, 1]
    elif chaser_states.ndim == 2 and chaser_states.shape[1] == 6:
        chaser_x = chaser_states[:, 0]
        chaser_y = chaser_states[:, 1]
    else:
        chaser_x = chaser_states[:, 0]
        chaser_y = chaser_states[:, 1]

    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot trajectory if requested
    if show_trajectory:
        ax.plot(chaser_x, chaser_y, 'b-', alpha=0.6, label='Chaser trajectory', linewidth=1)

    # Plot final position
    final_x = chaser_x[-1]
    final_y = chaser_y[-1]
    ax.plot(final_x, final_y, 'ro', markersize=8, label='Final position')

    # Plot target if requested
    if show_target:
        ax.plot(0, 0, 'yo', markersize=10, label='Target', zorder=5)

    # Plot initial position
    if chaser_states.shape[0] > 0:
        ax.plot(chaser_x[0], chaser_y[0], 'go', markersize=8, label='Initial position')

    # Formatting
    ax.set_xlabel('Radial distance (m) - x axis')
    ax.set_ylabel('Along-track distance (m) - y axis')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.legend()
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='-', alpha=0.3)

    # Set limits based on data with some margin
    all_x = [0] + list(chaser_x)  # Include target at origin
    all_y = [0] + list(chaser_y)
    max_range_val = max(max(all_x), max(all_y)) - min(min(all_x), min(all_y))
    margin = max_range_val * 0.1
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    return fig


def plot_range_velocity_history(time, ranges, ranges_dot, velocities, title="Range and Velocity History"):
    """Plot range and velocity history over time."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Range plot
    ax1.plot(time, ranges, 'b-', linewidth=1, label='Range r(t)')
    ax1.plot(time, np.zeros_like(time), 'k--', alpha=0.5, label='Zero range')
    ax1.set_ylabel('Range r(t) (m)')
    ax1.set_title(title)
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Add range error annotation
    final_range_error = abs(ranges[-1])
    ax1.annotate(
        f'Final range error: {final_range_error:.1f} m',
        xy=(time[-1], ranges[-1]),
        xytext=(0.02, 0.95),
        textcoords='axes fraction',
        fontsize=10,
        color='red',
        ha='left',
        va='top',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
    )

    # Range rate plot
    ax2.plot(time, ranges_dot, 'r-', linewidth=1, label='Range rate rÌ‡(t)')
    ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Range rate rÌ‡(t) (m/s)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    return fig, ax1, ax2


def plot_velocity_history(time, speeds, title="Speed History"):
    """Plot speed (magnitude of velocity) over time."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(time, speeds, 'g-', linewidth=1, label='Speed |v(t)|')
    ax.axhline(y=np.mean(speeds), color='k', linestyle='--', alpha=0.5, label=f'Mean: {np.mean(speeds):.1f} m/s')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()
    return fig, ax


if __name__ == "__main__":
    # Run the main simulation with default parameters
    print("\n" + "="*60)
    print("MAIN SIMULATION RUN")
    print("="*60 + "\n")

    # Default: 2km behind, 0.1 m/s closing velocity
    results = run_simulation(
        initial_range=2000.0,
        initial_closing_vel=0.1,
        orbit_altitude=400e3,
        simulation_time=600.0,
        time_step=0.1,
        navigation_constant=4.0,
        guidance_constant=10.0,
        thrust_magnitude=0.5,
        use_numerical_integration=True,
    )

    # Run parameter sweep
    print("\n\n" + "="*60)
    print("RUNNING PARAMETER SWEEP")
    print("="*60 + "\n")
    sweep_results = run_parameter_sweep()

    print("\n\nSimulation complete!")