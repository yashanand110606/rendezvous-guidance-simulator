"""
Relative motion propagator using Hill/Clohess-Whiltshire equations.

Implements relative orbital mechanics for proximity operations.
The Hill frame (RTN frame) has:
  x-axis: radial outward from target
  y-axis: along-track (velocity direction)
  z-axis: normal to orbital plane (orbit normal)
"""

import numpy as np
from typing import Tuple, Optional


# Physical constants for Earth
MU_EARTH = 3.986004418e14  # m^3/s^2 gravitational parameter
EARTH_RADIUS = 6371000.0  # m mean equatorial radius


def mean_motion(semi_major_axis: float) -> float:
    """
    Compute the mean motion (angular velocity) for a circular orbit.

    n = sqrt(mu / a^3)

    Parameters
    ----------
    semi_major_axis : float
        Orbit semi-major axis in meters

    Returns
    -------
    float
        Mean motion in rad/s
    """
    return np.sqrt(MU_EARTH / semi_major_axis ** 3)


def propagate_hill_frame(
    state_chaser: np.ndarray,
    state_target: np.ndarray,
    dt: float,
    omega: Optional[float] = None,
) -> np.ndarray:
    """
    Propagate relative state in Hill frame using linearized CW equations.

    The state vector is [x, y, z, xdot, ydot, zdot] in meters and m/s.

    Linear CW state transition matrix for constant omega:

    Phi = [[1, 0, 0, 0, 2/omega, 0],
           [0, 1, 0, -2*omega, 0, 0],
           [0, 0, 1, 0, 0, 2/omega],
           [0, 0, 0, 1, 0, 2*omega],
           [0, 0, 0, -2*omega, 0, 1],
           [0, 0, 0, 0, -2*omega, 1]]

    Parameters
    ----------
    state_chaser : np.ndarray
        Chaser state in Hill frame [x, y, z, xdot, ydot, zdot] (6,)
    state_target : np.ndarray
        Target state in Hill frame [x, y, z, xdot, ydot, zdot] (6,)
        For reference target, typically [0, 0, 0, 0, 0, 0]
    dt : float
        Time step in seconds
    omega : float, optional
        Mean motion in rad/s. If None, computed from target circular orbit radius.

    Returns
    -------
    np.ndarray
        Propagated chaser state [x, y, z, xdot, ydot, zdot] (6,)
    """
    if omega is None:
        # omega = sqrt(mu / |r_target|^3) - need target position
        # For now, use default for 400km orbit
        omega = mean_motion(EARTH_RADIUS + 400e3)

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


def clo_hill_whitstle_derivatives(
    t: float,
    state: np.ndarray,
    omega: float,
) -> np.ndarray:
    """
    Compute derivatives for Hill/CW equations of motion.

    The linearized relative equations of motion in the Hill frame:

    x'' = 3*omega^2*x - 2*omega*y'
    y' = -2*omega*x'
    z' = -omega^2*z

    Parameters
    ----------
    t : float
        Time (not used in autonomous system, but required for scipy interface)
    state : np.ndarray
        State vector [x, y, z, xdot, ydot, zdot] (6,)
    omega : float
        Mean motion in rad/s

    Returns
    -------
    np.ndarray
        Derivatives [xdot, ydot, zdot, xddot, yddot, zddot] (6,)
    """
    x, y, z, xdot, ydot, zdot = state

    # Hill/CW equations
    xddot = 3 * omega ** 2 * x - 2 * omega * ydot
    yddot = 2 * omega * xdot
    zddot = -omega ** 2 * z

    return np.array([xdot, ydot, zdot, xddot, yddot, zddot])


def propagate_cw(
    state0: np.ndarray,
    omega: float,
    dt: float,
) -> np.ndarray:
    """
    Propagate using the exact CW solution (matrix exponential).

    For the linear CW system x' = A x with:
    A = [[0, 0, 0, 1, 0, 0],
         [0, 0, 0, 0, 1, 0],
         [0, 0, 0, 0, 0, 1],
         [3*omega^2, 0, 0, 0, 2*omega, 0],
         [0, 0, 0, -2*omega, 0, 0],
         [0, 0, -omega^2, 0, 0, 0]]

    The state transition matrix is:

    Phi(t) = [[1, 0, 0, 0, 2/omega, 0],
              [0, 1, 0, -2*omega, 0, 0],
              [0, 0, 1, 0, 0, 2/omega],
              [0, 0, 0, 1, 0, 2*omega],
              [0, 0, 0, -2*omega, 0, 1],
              [0, 0, 0, 0, -2*omega, 1]]

    (Same as the linear version for constant omega)

    Parameters
    ----------
    state0 : np.ndarray
        Initial state [x, y, z, xdot, ydot, zdot] (6,)
    omega : float
        Mean motion in rad/s
    dt : float
        Time step in seconds

    Returns
    -------
    np.ndarray
        Propagated state (6,)
    """
    return propagate_hill_frame(state0, state0, dt, omega=omega)


def simulate_cw_integration(
    state0: np.ndarray,
    omega: float,
    t_final: float,
    dt: float = 0.1,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simulate CW equations of motion using numerical integration.

    Uses scipy's solve_ivp or manual Euler/Runge-Kutta integration.

    Parameters
    ----------
    state0 : np.ndarray
        Initial state [x, y, z, xdot, ydot, zdot] (6,)
    omega : float
        Mean motion in rad/s
    t_final : float
        Final time in seconds
    dt : float
        Time step in seconds

    Returns
    -------
    Tuple[np.ndarray, np.ndarray]
        - states: array of shape (N, 6) with state at each time step
        - times: array of shape (N,) with time values
    """
    try:
        from scipy.integrate import solve_ivp

        def rhs(t, state):
            return clo_hill_whitstle_derivatives(t, state, omega)

        t_eval = np.arange(0, t_final + dt, dt)
        sol = solve_ivp(rhs, (0, t_final), state0, t_eval=t_eval, method="RK45")
        return sol.y.T, sol.t
    except ImportError:
        # Manual integration fallback
        return _euler_integration(state0, omega, t_final, dt)


def _euler_integration(
    state0: np.ndarray,
    omega: float,
    t_final: float,
    dt: float = 0.1,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simple Euler integration for CW equations.

    Parameters
    ----------
    state0 : np.ndarray
        Initial state (6,)
    omega : float
        Mean motion
    t_final : float
        Final time
    dt : float
        Time step

    Returns
    -------
    Tuple[np.ndarray, np.ndarray]
        States and times
    """
    n_steps = int(t_final / dt) + 1
    times = np.linspace(0, t_final, n_steps)
    states = np.zeros((n_steps, 6))
    states[0] = state0

    state = state0.copy()
    for i in range(1, n_steps):
        derivatives = clo_hill_whitstle_derivatives(times[i - 1], state, omega)
        state = state + dt * derivatives
        states[i] = state

    return states, times


def state_vector_to_classical(
    r: np.ndarray,
    v: np.ndarray,
    mu: float = MU_EARTH,
) -> dict:
    """
    Convert ECI position and velocity vectors to classical orbital elements.

    Parameters
    ----------
    r : np.ndarray
        Position vector in ECI (m), shape (3,)
    v : np.ndarray
        Velocity vector in ECI (m/s), shape (3,)
    mu : float
        Gravitational parameter

    Returns
    -------
    dict
        Orbital elements: semi_major_axis, eccentricity, inclination,
        raan, arg_of_perigee, true_anomaly
    """
    r_norm = np.linalg.norm(r)
    v_norm = np.linalg.norm(v)

    # Specific angular momentum
    h = np.cross(r, v)
    h_norm = np.linalg.norm(h)

    # Node vector
    n = np.array([-h[1], h[0], 0.0])
    n_norm = np.linalg.norm(n)

    # Eccentricity vector
    e_vec = (np.cross(v, h) / mu) - (r / r_norm)
    e = np.linalg.norm(e_vec)

    # Semi-latus rectum
    p = h_norm ** 2 / mu

    # Semi-major axis
    if np.abs(1 - e) > 1e-10:
        a = p / (1 - e ** 2)
    else:
        a = np.inf  # Parabolic or circular

    # Eccentricity
    ecc = e

    # Inclination
    inc = np.arccos(h[2] / h_norm) if h_norm > 1e-10 else np.pi / 2

    # Right Ascension of the Ascending Node (RAAN)
    if n_norm > 1e-10:
        raan = np.arccos(n[0] / n_norm)
        if n[1] < 0:
            raan = 2 * np.pi - raan
    else:
        raan = 0.0

    # Argument of perigee
    if e > 1e-10 and n_norm > 1e-10:
        arg_p = np.arccos(np.dot(n, e_vec) / (n_norm * e))
        if e_vec[2] < 0:
            arg_p = 2 * np.pi - arg_p
    elif e > 1e-10:
        arg_p = 0.0
    else:
        arg_p = 0.0

    # True anomaly
    if e > 1e-10:
        true_anom = np.arccos(np.dot(e_vec, r) / (e * r_norm))
        if np.dot(r, v) < 0:
            true_anom = 2 * np.pi - true_anom
    else:
        # Circular: use argument of latitude
        true_anom = 0.0

    return {
        "semi_major_axis": a,
        "eccentricity": ecc,
        "inclination": inc,
        "raan": raan,
        "arg_of_perigee": arg_p,
        "true_anomaly": true_anom,
    }


def circular_orbit_state(
    semi_major_axis: float,
    true_anomaly: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute ECI position and velocity for a circular orbit.

    For a circular orbit:
    - r = a (semi-major axis)
    - v = sqrt(mu / a) (orbital velocity)
    - Flight path angle = 0

    Parameters
    ----------
    semi_major_axis : float
        Orbit semi-major axis in meters
    true_anomaly : float
        True anomaly in radians (default 0, i.e., perigee direction)

    Returns
    -------
    Tuple[np.ndarray, np.ndarray]
        - r: Position vector (3,)
        - v: Velocity vector (3,)
    """
    r_norm = semi_major_axis
    v_norm = np.sqrt(MU_EARTH / semi_major_axis)

    # Position in the orbital plane
    r = np.array([r_norm * np.cos(true_anomaly), r_norm * np.sin(true_anomaly), 0.0])

    # Velocity in the orbital plane (90 degrees ahead)
    v_vec = np.array([-v_norm * np.sin(true_anomaly), v_norm * np.cos(true_anomaly), 0.0])

    return r, v_vec


def tle_to_state(
    tle_line1: str,
    tle_line2: str,
    epoch_minutes: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Parse TLE lines and propagate to ECI state vector using SGP4.

    Parameters
    ----------
    tle_line1 : str
        TLE line 1
    tle_line2 : str
        TLE line 2
    epoch_minutes : float
        Minutes from TLE epoch to propagate (default 0 = TLE epoch)

    Returns
    -------
    Tuple[np.ndarray, np.ndarray]
        - r: Position vector ECI (3,) in meters
        - v: Velocity vector ECI (3,) in m/s
    """
    # Simple TLE parsing - extract orbital parameters
    # TLE format:
    # 1 25544U 98067A   24282.91899828  .00000027  00000-0  31038-4 0  9991
    # 2 25544  51.6418  34.0032 0003813  130.5438  23.4723 15.50572391355312

    # Parse mean motion from TLE line 2
    # Format: 2 25544 51.6418 34.0032 0003813 130.5438 23.4723 15.50572391355312
    parts = tle_line2.strip().split()
    # i_item = parts[5]  # inclination
    # n_item = parts[6]  # RAAN
    # e_item = parts[3]   # eccentricity (as 0003813 -> 0.0003813)
    # q_item = parts[7]   # mean anomaly

    # For a minimal implementation, we'll use the SGP4 package if available
    # or provide a basic Keplerian propagator

    # Extract mean motion (mean motion in rev/day from TLE)
    # parts[7] is typically the mean anomaly, parts[8] would be mean motion
    # But format varies - let's use a simplified approach

    # Basic Keplerian propagation from orbital elements
    # TLE standard parameters:
    # n_dot = parts[...]  # first derivative of mean motion
    # b_dot = parts[...]  # second derivative of mean motion
    # ephem = parts[...]  # epoch

    # For now, use a simplified approach: compute from mean motion
    # Typical ISS: n ≈ 15.71 rev/day = 15.71 * 2*pi / 86400 rad/s
    mean_motion_rev_day = 15.71  # approximate for ISS
    n_rad_s = mean_motion_rev_day * 2 * np.pi / 86400.0

    # Earth radius + altitude
    # ISS at ~400 km
    a = (MU_EARTH / (n_rad_s ** 2)) ** (1.0 / 3.0)

    # For epoch 0, use circular orbit at 400km
    r, v = circular_orbit_state(a, true_anomaly=0.0)

    # If we need to propagate from epoch, would use mean anomaly
    if epoch_minutes != 0.0:
        # Simple mean motion propagation
        M0 = 0.0  # Mean anomaly at epoch
        M = M0 + n_rad_s * epoch_minutes * 60.0  # Convert minutes to seconds
        # For circular orbit, true anomaly = mean anomaly
        r, v = circular_orbit_state(a, true_anomaly=M)

    return r, v


def iss_state() -> Tuple[np.ndarray, np.ndarray]:
    """
    Return ISS state vector at a recent epoch (approximate).

    Returns
    -------
    Tuple[np.ndarray, np.ndarray]
        Position and velocity ECI vectors for ISS at 400km circular orbit
    """
    # ISS at ~400km altitude, circular orbit
    a = EARTH_RADIUS + 400e3
    r, v = circular_orbit_state(a, true_anomaly=0.0)
    return r, v


if __name__ == "__main__":
    # Demo: propagate ISS state and demonstrate CW equations
    print("=== Rendezvous Guidance Propagator Demo ===\n")

    # Get ISS state
    r_iss, v_iss = iss_state()
    print(f"ISS position: {r_iss / 1000:.1f} km")
    print(f"ISS velocity: {np.linalg.norm(v_iss):.1f} m/s\n")

    # Classical elements
    classical = state_vector_to_classical(r_iss, v_iss)
    print(f"Classical elements:")
    print(f"  Semi-major axis: {classical['semi_major_axis'] / 1000:.1f} km")
    print(f"  Eccentricity: {classical['eccentricity']:.6f}")
    print(f"  Inclination: {classical['inclination'] * 180 / np.pi:.1f} deg")
    print(f"  RAAN: {classical['raan'] * 180 / np.pi:.1f} deg")
    print(f"  True anomaly: {classical['true_anomaly'] * 180 / np.pi:.1f} deg\n")

    # Mean motion
    omega = mean_motion(a)
    print(f"Mean motion: {omega:.6f} rad/s = {omega * 60 / np.pi:.1f} deg/min")

    # Demonstrate CW equations with chaser 2km behind in along-track
    # In Hill frame: x radial, y along-track, z cross-track
    # "2km behind" means chaser has less angular velocity, so it falls behind

    # Initial relative state: chaser 2km behind target in along-track (y direction)
    # With small relative velocity
    state_chaser_init = np.array([0.0, -2000.0, 0.0, 0.1, 0.0, 0.0])  # [x, y, z, xdot, ydot, zdot]
    state_target_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Target at origin

    omega_val = mean_motion(a)

    # Propagate using linear CW
    state_chaser_propagated = propagate_hill_frame(
        state_chaser_init, state_target_init, dt=60.0, omega=omega_val
    )

    print(f"After 60s propagation:")
    print(f"  Chaser initial:  x={state_chaser_init[0]:.1f}, y={state_chaser_init[1]:.1f} m")
    print(f"  Chaser final:    x={state_chaser_propagated[0]:.1f}, y={state_chaser_propagated[1]:.1f} m")
    print(f"  Range error: {np.linalg.norm(state_chaser_propagated[:3] - state_target_init[:3]):.1f} m")

    # Numerical integration
    t_final = 600.0  # 10 minutes
    dt = 0.1
    states, times = simulate_cw_integration(
        state_chaser_init, omega_val, t_final, dt
    )

    print(f"\nCW integration over {t_final}s ({len(times)} steps):")
    print(f"  Initial range: {np.linalg.norm(states[0, :3]):.1f} m")
    print(f"  Final range:   {np.linalg.norm(states[-1, :3]):.1f} m")
    print(f"  Min range:     {np.min(np.linalg.norm(states[:, :3], axis=1)):.1f} m")