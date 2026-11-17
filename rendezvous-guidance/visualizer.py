"""
2D/3D orbit visualization for rendezvous guidance simulation.

Provides plotting functions for:
- 2D relative motion (Hill frame RTN)
- 3D orbit trajectories
- Range vs time history
- Velocity vs time history
- GIF animation of approach
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import Tuple, Optional, List


def plot_2d_hill_frame(
    target_state: np.ndarray,
    chaser_states: np.ndarray,
    time: np.ndarray,
    title: str = "2D Rendezvous - Hill Frame",
    show_target: bool = True,
    show_trajectory: bool = True,
) -> plt.Axes:
    """
    Plot 2D relative motion in Hill frame (x-y plane).

    Parameters
    ----------
    target_state : np.ndarray
        Target state [x, y, z, xdot, ydot, zdot] (6,) - typically [0,0,0,0,0,0]
    chaser_states : np.ndarray
        Chaser states over time, shape (N, 6) or (N, 3) for positions only
    time : np.ndarray
        Time array (s)
    title : str
        Plot title
    show_target : bool
        Whether to mark target position
    show_trajectory : bool
        Whether to show chaser trajectory

    Returns
    -------
    plt.Axes
        Matplotlib axes object
    """
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
    max_range = max(max(all_x), max(all_y)) - min(min(all_x), min(all_y))
    margin = max_range * 0.1
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    return ax


def plot_range_velocity_history(
    time: np.ndarray,
    ranges: np.ndarray,
    ranges_dot: np.ndarray,
    velocities: np.ndarray,
    title: str = "Range and Velocity History",
) -> Tuple[plt.Figure, plt.Axes, plt.Axes]:
    """
    Plot range and velocity history over time.

    Parameters
    ----------
    time : np.ndarray
        Time array (s)
    ranges : np.ndarray
        Range values over time (m)
    ranges_dot : np.ndarray
        Range rate over time (m/s)
    velocities : np.ndarray
        Speed magnitude over time (m/s)

    Returns
    -------
    Tuple[plt.Figure, plt.Axes, plt.Axes]
        Figure and two axes objects (range ax, velocity ax)
    """
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


def plot_velocity_history(
    time: np.ndarray,
    speeds: np.ndarray,
    title: str = "Speed History",
) -> Tuple[plt.Figure, plt.Axes]:
    """
    Plot speed (magnitude of velocity) over time.

    Parameters
    ----------
    time : np.ndarray
        Time array (s)
    speeds : np.ndarray
        Speed values over time (m/s)

    Returns
    -------
    Tuple[plt.Figure, plt.Axes]
        Figure and axes objects
    """
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(time, speeds, 'g-', linewidth=1, label='Speed |v(t)|')
    ax.axhline(y=np.mean(speeds), color='k', linestyle='--', alpha=0.5, label=f'Mean: {np.mean(speeds):.1f} m/s')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()
    return fig, ax


def plot_3d_orbits(
    target_positions: np.ndarray,
    chaser_positions: np.ndarray,
    title: str = "3D Rendezvous Trajectories",
) -> plt.Axes:
    """
    Plot 3D orbit trajectories.

    Parameters
    ----------
    target_positions : np.ndarray
        Target positions over time, shape (N, 3)
    chaser_positions : np.ndarray
        Chaser positions over time, shape (N, 3)
    title : str
        Plot title

    Returns
    -------
    plt.Axes
        3D matplotlib axes object
    """
    fig = plt.figure(figsize=(10, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Plot target trajectory
    if np.any(target_positions):
        ax.plot(
            target_positions[:, 0],
            target_positions[:, 1],
            target_positions[:, 2],
            'yo-', markersize=3, linewidth=1, alpha=0.7, label='Target trajectory'
        )

    # Plot chaser trajectory
    if np.any(chaser_positions):
        ax.plot(
            chaser_positions[:, 0],
            chaser_positions[:, 1],
            chaser_positions[:, 2],
            'b-', linewidth=2, alpha=0.8, label='Chaser trajectory'
        )

    # Mark initial and final positions
    if chaser_positions.shape[0] > 0:
        ax.plot(
            chaser_positions[0, 0], chaser_positions[0, 1], chaser_positions[0, 2],
            'go', markersize=8, label='Chaser start'
        )
        ax.plot(
            chaser_positions[-1, 0], chaser_positions[-1, 1], chaser_positions[-1, 2],
            'ro', markersize=8, label='Chaser end'
        )

    # Mark target start/end
    if np.any(target_positions):
        ax.plot(
            target_positions[0, 0], target_positions[0, 1], target_positions[0, 2],
            'mo', markersize=8, label='Target start'
        )
        ax.plot(
            target_positions[-1, 0], target_positions[-1, 1], target_positions[-1, 2],
            'cyan', markersize=8, label='Target end'
        )

    # Formatting
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Set equal aspect ratio - for 3D this is approximate
    all_x = np.concatenate([target_positions[:, 0], chaser_positions[:, 0]])
    all_y = np.concatenate([target_positions[:, 1], chaser_positions[:, 1]])
    all_z = np.concatenate([target_positions[:, 2], chaser_positions[:, 2]])

    if len(all_x) > 0:
        max_range = max(np.max(all_x) - np.min(all_x),
                        np.max(all_y) - np.min(all_y),
                        np.max(all_z) - np.min(all_z))
        mean_range = (np.max(all_x) + np.min(all_x) +
                      np.max(all_y) + np.min(all_y) +
                      np.max(all_z) + np.min(all_z)) / 6
        ax.set_xlim(mean_range - max_range/2, mean_range + max_range/2)
        ax.set_ylim(mean_range - max_range/2, mean_range + max_range/2)
        ax.set_zlim(mean_range - max_range/2, mean_range + max_range/2)

    return ax


def animate_approach(
    time: np.ndarray,
    chaser_positions: np.ndarray,
    target_positions: np.ndarray,
    output_path: Optional[str] = None,
    interval: int = 20,
) -> animation.FuncAnimation:
    """
    Create animation of the rendezvous approach.

    Parameters
    ----------
    time : np.ndarray
        Time array (s)
    chaser_positions : np.ndarray
        Chaser positions over time, shape (N, 3)
    target_positions : np.ndarray
        Target positions over time, shape (N, 3)
    output_path : str, optional
        If provided, save GIF to this path
    interval : int
        Animation interval in milliseconds

    Returns
    -------
    animation.FuncAnimation
        Matplotlib animation object
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # Set up axis limits
    all_pos = np.concatenate([chaser_positions, target_positions])
    max_range = np.max(np.abs(all_pos)) * 1.1
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Rendezvous Approach Animation')
    ax.grid(True, alpha=0.3)

    # Initialize plots
    target_dot, = ax.plot([], [], [], 'yo', markersize=12, label='Target')
    chaser_dot, = ax.plot([], [], [], 'bo', markersize=8, label='Chaser')
    trail_line, = ax.plot([], [], [], 'b-', alpha=0.5, linewidth=1)
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes,
                        fontsize=10, verticalalignment='top')

    ax.legend(loc='upper right')

    chaser_trail = []
    target_trail = []

    def init():
        target_dot.set_data([], [])
        target_dot.set_3d_properties([])
        chaser_dot.set_data([], [])
        chaser_dot.set_3d_properties([])
        trail_line.set_data([], [])
        trail_line.set_3d_properties([])
        time_text.set_text('')
        return target_dot, chaser_dot, trail_line, time_text

    def update(frame):
        # Update positions
        chaser_x = chaser_positions[frame, 0]
        chaser_y = chaser_positions[frame, 1]
        chaser_z = chaser_positions[frame, 2]
        target_x = target_positions[frame, 0]
        target_y = target_positions[frame, 1]
        target_z = target_positions[frame, 2]

        chaser_dot.set_data([chaser_x], [chaser_y])
        chaser_dot.set_3d_properties([chaser_z])
        target_dot.set_data([target_x], [target_y])
        target_dot.set_3d_properties([target_z])

        # Update trails
        chaser_trail.append((chaser_x, chaser_y, chaser_z))
        target_trail.append((target_x, target_y, target_z))

        if len(chaser_trail) > 1:
            trail_x = [t[0] for t in chaser_trail]
            trail_y = [t[1] for t in chaser_trail]
            trail_z = [t[2] for t in chaser_trail]
            trail_line.set_data(trail_x, trail_y)
            trail_line.set_3d_properties(trail_z)

        # Update time display
        time_text.set_text(f't = {time[frame]:.1f} s')

        return target_dot, chaser_dot, trail_line, time_text

    anim = animation.FuncAnimation(
        fig, update, frames=len(time),
        init_func=init, blit=False, interval=interval, repeat=False
    )

    if output_path:
        anim.save(output_path, writer='pillow', fps=50/duration)

    return anim


def plot_cw_phase_portrait(
    states: np.ndarray,
    title: str = "CW Phase Portrait (x vs y)",
) -> plt.Axes:
    """
    Plot phase portrait of CW equations: x vs y position.

    Parameters
    ----------
    states : np.ndarray
        State array (N, 6) with [x, y, z, xdot, ydot, zdot]
    title : str
        Plot title

    Returns
    -------
    plt.Axes
        Matplotlib axes object
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(states[:, 0], states[:, 1], 'b-', alpha=0.7, linewidth=1)
    ax.plot(states[0, 0], states[0, 1], 'go', markersize=8, label='Initial')
    ax.plot(states[-1, 0], states[-1, 1], 'ro', markersize=8, label='Final')

    ax.set_xlabel('Radial position x (m)')
    ax.set_ylabel('Along-track position y (m)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.legend()

    return ax


def save_figure(
    fig: plt.Figure,
    filename: str,
    dpi: int = 150,
    tight_layout: bool = True,
) -> None:
    """
    Save matplotlib figure in multiple formats.

    Parameters
    ----------
    fig : plt.Figure
        Figure to save
    filename : str
        Base filename (without extension will add .png and .pdf)
    dpi : int
        DPI for raster output
    tight_layout : bool
        Whether to apply tight_layout before saving
    """
    if tight_layout:
        fig.tight_layout()

    # Save as PNG
    png_path = f"{filename}.png"
    fig.savefig(png_path, dpi=dpi, bbox_inches='tight')
    print(f"Saved: {png_path}")

    # Save as PDF
    pdf_path = f"{filename}.pdf"
    fig.savefig(pdf_path, format='pdf', bbox_inches='tight')
    print(f"Saved: {pdf_path}")


def close_all_figures():
    """Close all matplotlib figures to free memory."""
    plt.close('all')


if __name__ == "__main__":
    # Quick demo of visualization functions
    print("=== Visualization Demo ===\n")

    # Generate some sample data
    np.random.seed(42)
    n_points = 100
    time = np.linspace(0, 600, n_points)

    # Simulate range decreasing from 2000m to ~0
    ranges = 2000 * (1 - np.linspace(0, 1, n_points)) + np.random.normal(0, 5, n_points)
    ranges_dot = -2000 / 600 + np.random.normal(0, 0.1, n_points)  # Approximately constant rate
    speeds = np.sqrt(0.01**2 + np.random.normal(0, 0.05, n_points)**2) + 0.1

    # 2D Hill frame plot
    from rendezvous_guidance.propagator import propagate_hill_frame, mean_motion, EARTH_RADIUS
    from rendezvous_guidance.controller import ProportionalNavigationController

    # Initial relative state: 2km behind in along-track
    state_init = np.array([0.0, -2000.0, 0.0, 0.1, 0.0, 0.0])
    omega = mean_motion(EARTH_RADIUS + 400e3)

    # Propagate
    chaser_states = np.zeros((n_points, 6))
    chaser_states[0] = state_init
    for i in range(1, n_points):
        chaser_states[i] = propagate_hill_frame(
            chaser_states[i-1], chaser_states[i-1], dt=0.1, omega=omega
        )

    ax = plot_2d_hill_frame(
        target_state=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        chaser_states=chaser_states,
        time=time,
        title="CW Propagation: Chaser 2km Behind"
    )
    print("2D Hill frame plot created âœ“")

    # Range and velocity history
    fig, ax_range, ax_vel = plot_range_velocity_history(
        time, ranges, ranges_dot, speeds,
        title="Rendezvous Guidance: Range & Velocity"
    )
    print("Range/velocity plot created âœ“")

    # Speed history
    fig2, ax_speed = plot_velocity_history(time, speeds, title="Chaser Speed")
    print("Speed plot created âœ“")

    # 3D trajectories (sampled)
    target_pos = np.column_stack([
        np.random.normal(0, 100, n_points),
        np.random.normal(0, 100, n_points),
        np.random.normal(0, 100, n_points),
    ])
    chaser_pos = chaser_states[:, :3]

    ax3d = plot_3d_orbits(target_positions=target_pos, chaser_positions=chaser_pos)
    print("3D orbit plot created âœ“")

    # Phase portrait
    ax_phase = plot_cw_phase_portrait(chaser_states, title="CW Phase Portrait")
    print("Phase portrait created âœ“")

    # Save figures
    save_fig = True
    if save_fig:
        save_figure(fig, "rendezvous_range_velocity")
        save_figure(fig2, "rendezvous_speed")
        save_figure(fig3 := plt.gcf(), "rendezvous_3d_orbits") if 'fig3' in dir() else None
        print("Figures saved âœ“")

    plt.show()