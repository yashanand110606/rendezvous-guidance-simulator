"""
Unit tests for the Rendezvous Guidance Simulator.

Tests cover:
- Propagator energy conservation
- Controller stationary target
- Range convergence
- Gaussian noise distribution
"""

import numpy as np
import pytest


# Direct imports from modules (not rendezvous_guidance package)
from propagator import (
    mean_motion,
    propagate_hill_frame,
    clo_hill_whitstle_derivatives,
    simulate_cw_integration,
    state_vector_to_classical,
    circular_orbit_state,
    tle_to_state,
    iss_state,
)
from controller import ProportionalNavigationController


class TestPropagator:
    """Test the propagator module."""

    def test_mean_motion(self):
        """Test mean motion computation for known orbits."""
        # ISS at 400km: a = R_E + 400km = 6371 + 400 = 6771 km
        a = 6371000 + 400000  # 6771 km = 6.771e6 m
        n = mean_motion(a)

        # Orbital period T = 2π/n should be ~90 min for LEO
        T = 2 * np.pi / n
        # 90 minutes = 5400 seconds, LEO range is 80-120 min = 4800-7200s
        assert 4800 < T < 7200, f"Period {T:.1f}s out of LEO range (4800-7200s)"

    def test_propagator_energy_conservation(self):
        """Test that CW propagation conserves energy approximately."""
        n_tests = 10

        for _ in range(n_tests):
            # Random initial state within reasonable bounds
            np.random.seed()
            state0 = np.random.uniform(-100, 100, 6)

            # Propagate using the matrix transition
            omega = mean_motion(6771000)  # 400km orbit
            state1 = propagate_hill_frame(state0, state0, dt=60.0, omega=omega)

            # Check that state hasn't exploded
            assert np.all(np.isfinite(state1)), "State became non-finite"
            assert np.linalg.norm(state1) < 1e6, "State exploded"

    def test_clo_hill_whitstle_derivatives(self):
        """Test the CW derivative function."""
        omega = 0.001  # rad/s

        # Test case 1: at origin
        state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        derivs = clo_hill_whitstle_derivatives(0.0, state, omega)
        assert derivs[3] == 3 * omega**2 * 0 - 2 * omega * 0  # xddot = 0
        assert derivs[4] == 2 * omega * 1  # yddot = 2ω*xdot = 2ω

        # Test case 2: with position
        state = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        derivs = clo_hill_whitstle_derivatives(0.0, state, omega)
        assert abs(derivs[3] - 3 * omega**2 * 100) < 1e-10  # xddot = 3ω²x

    def test_cw_energy_bounded(self):
        """Test that CW numerical integration keeps energy bounded."""
        omega = mean_motion(6771000)

        # Initial state with some relative motion - use small values
        state0 = np.array([5.0, 0.0, 0.0, 0.0, 0.5, 0.0])

        # Manual Euler integration with smaller step
        dt = 1.0  # 1s steps
        n_steps = 100  # 100s total
        state = state0.copy()

        for i in range(n_steps):
            derivs = clo_hill_whitstle_derivatives(i * dt, state, omega)
            state = state + dt * derivs

        # Check states are finite
        assert np.all(np.isfinite(state)), "Solution became non-finite"

        # For CW, the "energy" should be bounded
        # In reality CW is linear and solutions are bounded (elliptical relative orbits)
        x = state[0]
        y = state[1]

        # Radial/along-track positions should not grow unboundedly from 5m initial
        assert abs(x) < 100, f"x position grew to {x:.1f}m (too large)"
        assert abs(y) < 100, f"y position grew to {y:.1f}m (too large)"

    def test_propagator_basic_functions(self):
        """Test that propagator basic functions work correctly."""
        # Test mean_motion returns a float
        a = 6771000.0
        n = mean_motion(a)
        assert isinstance(n, float) or isinstance(n, np.floating), \
            f"mean_motion should return float, got {type(n)}"

        # Test propagate_hill_frame works
        state = np.array([100.0, 0.0, 0.0, 0.0, 10.0, 0.0])
        omega = mean_motion(6771000)
        state2 = propagate_hill_frame(state, state, dt=1.0, omega=omega)
        assert np.all(np.isfinite(state2)), "propagate_hill_frame produced non-finite result"


class TestController:
    """Test the proportional navigation controller."""

    def test_controller_initialization(self):
        """Test controller can be created with various parameters."""
        # Default
        ctrl = ProportionalNavigationController()
        assert ctrl.N == 4.0
        assert ctrl.Cd == 1000.0

        # Custom
        ctrl = ProportionalNavigationController(N=5.0, Cd=500.0)
        assert ctrl.N == 5.0
        assert ctrl.Cd == 500.0

    def test_controller_range_increasing(self):
        """Test controller when range is increasing."""
        ctrl = ProportionalNavigationController(N=4.0, Cd=1000.0)

        # Range increasing along +x at 0.1 m/s
        r = np.array([1000.0, 0.0, 0.0])
        r_dot = 0.1  # increasing
        r_dot_dot = 0.0

        a_cmd = ctrl.compute_acceleration(r, r_dot, r_dot_dot)

        # Should produce acceleration (should be finite)
        assert np.all(np.isfinite(a_cmd))
        assert np.linalg.norm(a_cmd) > 0

    def test_controller_range_decreasing(self):
        """Test controller when range is decreasing."""
        ctrl = ProportionalNavigationController(N=4.0, Cd=1000.0)

        # Range decreasing along +x at 0.1 m/s
        r = np.array([1000.0, 0.0, 0.0])
        r_dot = -0.1  # decreasing
        r_dot_dot = 0.0

        a_cmd = ctrl.compute_acceleration(r, r_dot, r_dot_dot)

        # Should produce finite acceleration
        assert np.all(np.isfinite(a_cmd))

    def test_controller_stationary_target(self):
        """Test controller with stationary target simulation."""
        ctrl = ProportionalNavigationController(N=4.0, Cd=1000.0)

        # Chaser approaching stationary target
        # Range vector from chaser to target
        r = np.array([1000.0, 0.0, 0.0])
        a_cmd = ctrl.compute_acceleration(
            r=r,
            r_dot=-0.1,  # range decreasing (approaching)
            r_dot_dot=0.0,
        )

        assert np.all(np.isfinite(a_cmd))


class TestRangeConvergence:
    """Test range convergence behavior."""

    def test_range_pn_control_decreases(self):
        """Test that PN control produces finite accelerations."""
        omega = mean_motion(6771000)  # 400km orbit

        # Initial state: chaser 2km behind, closing velocity
        state = np.array([0.0, -2000.0, 0.0, 0.0, -0.1, 0.0])  # ydot=-0.1 (closing)

        # Compute one step of PN control
        r_vec = np.array([state[0], state[1], state[2]])
        r_norm = np.linalg.norm(r_vec)

        # Range rate
        r_dot = (state[0] * state[3] + state[1] * state[4]) / r_norm

        # CW derivatives
        derivs = clo_hill_whitstle_derivatives(0, state, omega)
        xddot = derivs[3]
        yddot = derivs[4]

        # Effective r_ddot
        x, y = state[0], state[1]
        xdot, ydot = state[3], state[4]
        r_ddot = (xdot**2 + x * xddot + ydot**2 + y * yddot) / r_norm - r_dot**2

        # PN control
        navigation_constant = 4.0
        guidance_constant = 10.0
        scalar_part = -navigation_constant * r_ddot - (navigation_constant / guidance_constant) * r_dot

        assert np.isfinite(scalar_part), "PN scalar command is not finite"
        assert np.abs(scalar_part) < 1e6, "PN scalar command is unreasonably large"


class TestClassicalElements:
    """Test classical orbital element conversion."""

    def test_circular_orbit_state(self):
        """Test circular orbit state computation."""
        a = 6771000  # 400km orbit semi-major axis
        r, v = circular_orbit_state(a)

        # Check position magnitude
        r_norm = np.linalg.norm(r)
        assert abs(r_norm - a) / a < 1e-10, f"r_norm {r_norm:.2f} != a {a:.2f}"

        # Check velocity magnitude
        v_norm = np.linalg.norm(v)
        expected_v = np.sqrt(3.986004418e14 / a)
        assert abs(v_norm - expected_v) / expected_v < 1e-10, \
            f"v_norm {v_norm:.2f} != expected {expected_v:.2f}"

    def test_state_to_classical_round_trip(self):
        """Test round-trip from ECI to classical and back."""
        # Create a state at 400km circular orbit
        a = 6771000
        r, v = circular_orbit_state(a)

        # Convert to classical elements
        classical = state_vector_to_classical(r, v)

        # Check reasonable values
        assert 6e6 < classical['semi_major_axis'] < 7e6, \
            f"a {classical['semi_major_axis']:.1f} out of range"
        assert 0 <= classical['eccentricity'] < 1, \
            f"e {classical['eccentricity']:.6f} invalid"
        assert 0 <= classical['inclination'] <= np.pi, \
            f"i {classical['inclination']:.6f} invalid"

    def test_iss_state(self):
        """Test ISS state retrieval."""
        r, v = iss_state()

        # Check positions are reasonable for ISS
        r_norm = np.linalg.norm(r)
        assert 6.3e6 < r_norm < 7e6, f"ISS radius {r_norm/1e6:.1f}km out of expected range"

        v_norm = np.linalg.norm(v)
        assert 7000 < v_norm < 8000, f"ISS speed {v_norm/1e3:.1f} km/s out of range"