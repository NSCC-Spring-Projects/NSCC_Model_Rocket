"""
Model Rocket Flight Simulator Framework

This module calculates flight characteristics for model rockets including:
- Maximum acceleration
- Maximum velocity
- Maximum altitude
- Time to key flight points
- Burn time, coast to apogee, descent, and total flight time
"""

from dataclasses import dataclass
from typing import Tuple
import math


@dataclass
class Motor:
    """Model rocket motor specifications"""
    name: str
    diameter_mm: float          # Motor diameter in mm
    length_mm: float            # Motor length in mm
    total_impulse: float        # Total impulse in N-sec
    thrust_duration: float      # Burn time in seconds
    max_thrust: float           # Maximum thrust in Newtons
    time_delay: float           # Time delay before ejection (seconds)
    max_lift_weight: float      # Maximum recommended weight in grams
    total_mass: float           # Motor total mass in grams
    propellant_mass: float      # Propellant mass in grams
    
    @property
    def burnout_mass(self) -> float:
        """Mass remaining after burnout (in grams)"""
        return self.total_mass - self.propellant_mass


@dataclass
class Rocket:
    """Model rocket specifications"""
    name: str
    length_mm: float            # Rocket length in mm
    diameter_mm: float          # Body diameter in mm
    mass: float                 # Dry mass in grams
    drag_coefficient: float     # Drag coefficient (0.5-0.75 typical)
    
    @property
    def cross_section_area(self) -> float:
        """Calculate cross-sectional area in m^2"""
        radius_m = self.diameter_mm / 2000  # Convert mm to m
        return math.pi * radius_m ** 2


class FlightSimulator:
    """Calculates model rocket flight characteristics using standard formulas"""
    
    def __init__(self, rocket: Rocket, motor: Motor):
        """
        Initialize flight simulator
        
        Args:
            rocket: Rocket instance
            motor: Motor instance
        """
        self.rocket = rocket
        self.motor = motor
        
        # Constants (SI units)
        self.g = 9.81  # m/s^2, gravity
        self.rho = 1.225  # kg/m^3, air density at sea level
        self.pi = 3.14159
        
        # Convert all inputs to SI units (kg, m, N, s)
        self.mr_kg = rocket.mass / 1000  # rocket mass in kg
        self.mm_kg = motor.total_mass / 1000  # motor total mass in kg
        self.mp_kg = motor.propellant_mass / 1000  # propellant mass in kg
        self.tb_s = motor.thrust_duration  # burn time in seconds
        
        # Calculated masses (motor total mass already includes propellant)
        # formula: m0 = mr + mm, mb = m0 - mp
        self.m0_kg = self.mr_kg + self.mm_kg  # initial mass
        self.mb_kg = self.m0_kg - self.mp_kg  # burnout mass

    def _reference_area_m2(self) -> float:
        """Calculate the rocket's frontal area in m^2."""
        D_m = self.rocket.diameter_mm / 1000
        return self.pi * (D_m / 2) ** 2

    def _burn_rate_kg_per_s(self) -> float:
        """Return the propellant burn rate in kg/s."""
        if self.tb_s <= 0:
            return 0.0
        return self.mp_kg / self.tb_s

    def _simulate_powered_flight(self, dt: float = 0.001) -> tuple[float, float, float, float]:
        """Integrate the powered phase with drag.

        Returns:
            burnout_velocity, burn_altitude, initial_acceleration, burnout_acceleration
        """
        if self.tb_s <= 0:
            return 0.0, 0.0, 0.0, 0.0

        thrust = self.calculate_average_thrust()
        burn_rate = self._burn_rate_kg_per_s()
        step = min(dt, self.tb_s / 1000)

        time_elapsed = 0.0
        velocity = 0.0
        altitude = 0.0

        initial_acceleration = (thrust - (self.m0_kg * self.g) - self.calculate_drag_force(0.0)) / self.m0_kg
        burnout_acceleration = initial_acceleration

        while time_elapsed < self.tb_s:
            current_step = min(step, self.tb_s - time_elapsed)
            current_mass = max(self.mb_kg, self.m0_kg - (burn_rate * time_elapsed))
            drag_force = self.calculate_drag_force(max(velocity, 0.0))
            acceleration = (thrust - (current_mass * self.g) - drag_force) / current_mass

            velocity = max(0.0, velocity + acceleration * current_step)
            altitude += velocity * current_step
            burnout_acceleration = acceleration
            time_elapsed += current_step

        return velocity, altitude, initial_acceleration, burnout_acceleration

    def _simulate_coast_to_apogee(self, burnout_velocity: float, dt: float = 0.001) -> tuple[float, float]:
        """Integrate the coast phase until vertical velocity reaches zero."""
        if burnout_velocity <= 0:
            return 0.0, 0.0

        time_elapsed = 0.0
        velocity = burnout_velocity
        altitude = 0.0

        step = dt
        while velocity > 0:
            drag_force = self.calculate_drag_force(velocity)
            acceleration = -(self.g + (drag_force / self.mb_kg))
            next_velocity = velocity + (acceleration * step)

            altitude += max(0.0, (velocity + max(next_velocity, 0.0)) / 2) * step
            velocity = next_velocity
            time_elapsed += step

            if velocity <= 0:
                break

        return time_elapsed, altitude

    def _simulate_descent(self, max_altitude: float, dt: float = 0.001) -> float:
        """Integrate the descent phase with drag until the rocket reaches the ground."""
        if max_altitude <= 0:
            return 0.0

        time_elapsed = 0.0
        altitude = max_altitude
        downward_speed = 0.0
        step = dt

        while altitude > 0:
            drag_force = self.calculate_drag_force(downward_speed)
            acceleration = self.g - (drag_force / self.mb_kg)
            downward_speed = max(0.0, downward_speed + (acceleration * step))
            altitude -= downward_speed * step
            time_elapsed += step

        return time_elapsed
    
    def calculate_average_thrust(self) -> float:
        """
        Calculate average thrust using: F_avg = It / tb
        
        Returns:
            Average thrust in Newtons
        """
        return self.motor.total_impulse / self.tb_s
    
    def calculate_initial_acceleration(self) -> float:
        """
        Calculate initial acceleration with drag: a0 = (F_avg - (m0 * g) - F_drag) / m0
        
        Returns:
            Initial acceleration in m/s^2
        """
        F_avg = self.calculate_average_thrust()
        drag_force = self.calculate_drag_force(0.0)
        a0 = (F_avg - (self.m0_kg * self.g) - drag_force) / self.m0_kg
        return a0
    
    def calculate_max_acceleration(self) -> float:
        """
        Calculate maximum acceleration near burnout with drag.
        
        Returns:
            Maximum acceleration in m/s^2
        """
        F_avg = self.calculate_average_thrust()
        burnout_velocity, _, _, _ = self._simulate_powered_flight()
        drag_force = self.calculate_drag_force(burnout_velocity)
        a_max = (F_avg - (self.mb_kg * self.g) - drag_force) / self.mb_kg
        return a_max
    
    def calculate_average_acceleration(self) -> float:
        """
        Calculate average acceleration: a_avg = (a0 + a_max) / 2
        
        Returns:
            Average acceleration in m/s^2
        """
        a0 = self.calculate_initial_acceleration()
        a_max = self.calculate_max_acceleration()
        a_avg = (a0 + a_max) / 2
        return a_avg
    
    def calculate_drag_force(self, velocity: float) -> float:
        """
        Calculate drag force: F_drag = 0.5 * rho * Cd * A * v^2
        
        Args:
            velocity: Velocity in m/s
            
        Returns:
            Drag force in Newtons
        """
        A = self._reference_area_m2()
        drag_force = 0.5 * self.rho * self.rocket.drag_coefficient * A * (velocity ** 2)
        return drag_force
    
    def calculate_velocity_at_burnout(self) -> float:
        """
        Calculate velocity at burnout using powered-flight integration with drag.
        
        Returns:
            Velocity at burnout in m/s
        """
        v_b, _, _, _ = self._simulate_powered_flight()
        return v_b
    
    def calculate_max_velocity(self) -> float:
        """
        Maximum velocity (approximately at burnout for small rockets)
        
        Returns:
            Maximum velocity in m/s
        """
        return self.calculate_velocity_at_burnout()
    
    def calculate_burn_altitude(self) -> float:
        """
        Calculate altitude during burn phase using powered-flight integration.
        
        Returns:
            Altitude at burnout in meters
        """
        _, h_b, _, _ = self._simulate_powered_flight()
        return h_b
    
    def calculate_coast_time(self, velocity_at_burnout: float) -> float:
        """
        Calculate coast time from burnout to apogee with drag.
        
        Args:
            velocity_at_burnout: Velocity at burnout in m/s
            
        Returns:
            Coast time in seconds
        """
        t_coast, _ = self._simulate_coast_to_apogee(velocity_at_burnout)
        return t_coast
    
    def calculate_coast_altitude(self, velocity_at_burnout: float) -> float:
        """
        Calculate altitude gained during coast with drag.
        
        Args:
            velocity_at_burnout: Velocity at burnout in m/s
            
        Returns:
            Altitude during coast phase in meters
        """
        _, h_coast = self._simulate_coast_to_apogee(velocity_at_burnout)
        return h_coast
    
    def calculate_max_altitude(self) -> float:
        """
        Calculate maximum altitude with drag-aware ascent phases.
        
        Returns:
            Maximum altitude in meters
        """
        v_b, h_b, _, _ = self._simulate_powered_flight()
        _, h_coast = self._simulate_coast_to_apogee(v_b)
        h_max = h_b + h_coast
        return h_max
    
    def calculate_terminal_velocity(self) -> float:
        """
        Calculate terminal velocity: v_terminal = sqrt((2 * m * g) / (rho * Cd * A))
        
        Uses burnout mass for descent phase
        
        Returns:
            Terminal velocity in m/s
        """
        A = self._reference_area_m2()
        
        numerator = 2 * self.mb_kg * self.g
        denominator = self.rho * self.rocket.drag_coefficient * A
        
        v_terminal = math.sqrt(numerator / denominator)
        return v_terminal
    
    def calculate_descent_time(self, max_altitude: float) -> float:
        """
        Calculate descent time with drag-aware descent integration.
        
        Args:
            max_altitude: Maximum altitude in meters
            
        Returns:
            Descent time in seconds
        """
        t_descent = self._simulate_descent(max_altitude)
        return t_descent
    
    def calculate_burn_time(self) -> float:
        """
        Burn time: t_burn = tb
        
        Returns:
            Burn time in seconds
        """
        return self.tb_s
    
    def calculate_total_ascent_time(self, coast_time: float) -> float:
        """
        Total ascent time: t_ascent = tb + t_coast
        
        Args:
            coast_time: Coast time in seconds
            
        Returns:
            Total ascent time in seconds
        """
        t_ascent = self.tb_s + coast_time
        return t_ascent
    
    def calculate_total_flight_time(self, coast_time: float, descent_time: float) -> float:
        """
        Total flight time: t_total = tb + t_coast + t_descent
        
        Args:
            coast_time: Coast time in seconds
            descent_time: Descent time in seconds
            
        Returns:
            Total flight time in seconds
        """
        t_total = self.tb_s + coast_time + descent_time
        return t_total

    def build_calculation_steps(self) -> list[tuple[str, float | str]]:
        """Build a readable step-by-step calculation trace."""
        D_m = self.rocket.diameter_mm / 1000
        A = self._reference_area_m2()

        F_avg = self.calculate_average_thrust()
        a0 = self.calculate_initial_acceleration()
        a_max = self.calculate_max_acceleration()
        a_avg = self.calculate_average_acceleration()
        v_b = self.calculate_velocity_at_burnout()
        h_b = self.calculate_burn_altitude()
        t_coast = self.calculate_coast_time(v_b)
        h_coast = self.calculate_coast_altitude(v_b)
        h_max = h_b + h_coast
        v_terminal = self.calculate_terminal_velocity()
        t_descent = self.calculate_descent_time(h_max)
        t_ascent = self.calculate_total_ascent_time(t_coast)
        t_total = self.calculate_total_flight_time(t_coast, t_descent)

        return [
            ("Constants", "g = 9.81 m/s², rho = 1.225 kg/m³, pi = 3.14159"),
            ("Reference area", f"A = pi * (D/2)^2 = 3.14159 * ({D_m:.5f}/2)^2 = {A:.6f} m²"),
            ("Initial mass", f"m0 = mr + mm = {self.mr_kg:.4f} + {self.mm_kg:.4f} = {self.m0_kg:.4f} kg"),
            ("Burnout mass", f"mb = m0 - mp = {self.m0_kg:.4f} - {self.mp_kg:.4f} = {self.mb_kg:.4f} kg"),
            ("Average thrust", f"F_avg = It / tb = {self.motor.total_impulse:.3f} / {self.tb_s:.3f} = {F_avg:.3f} N"),
            ("Initial acceleration", f"a0 = (F_avg - m0*g - F_drag) / m0 = {a0:.3f} m/s²"),
            ("Max acceleration", f"a_max = (F_avg - mb*g - F_drag) / mb = {a_max:.3f} m/s²"),
            ("Average acceleration", f"a_avg = (a0 + a_max) / 2 = {a_avg:.3f} m/s²"),
            ("Velocity at burnout", f"v_b = a_avg * tb = {a_avg:.3f} * {self.tb_s:.3f} = {v_b:.3f} m/s"),
            ("Burn altitude", f"h_b = 0.5 * a_avg * tb^2 = {h_b:.3f} m"),
            ("Coast time", f"t_coast = drag-aware integration = {t_coast:.3f} s"),
            ("Coast altitude", f"h_coast = drag-aware integration = {h_coast:.3f} m"),
            ("Maximum altitude", f"h_max = h_b + h_coast = {h_b:.3f} + {h_coast:.3f} = {h_max:.3f} m"),
            ("Terminal velocity", f"v_terminal = sqrt((2*m*g)/(rho*Cd*A)) = {v_terminal:.3f} m/s"),
            ("Descent time", f"t_descent = drag-aware integration = {t_descent:.3f} s"),
            ("Total ascent time", f"t_ascent = tb + t_coast = {self.tb_s:.3f} + {t_coast:.3f} = {t_ascent:.3f} s"),
            ("Total flight time", f"t_total = tb + t_coast + t_descent = {self.tb_s:.3f} + {t_coast:.3f} + {t_descent:.3f} = {t_total:.3f} s"),
        ]
    
    def run_simulation(self) -> dict:
        """
        Run complete flight simulation
        
        Returns:
            Dictionary with all calculated flight characteristics
        """
        # Calculate phase durations and key values
        v_b = self.calculate_velocity_at_burnout()
        t_coast = self.calculate_coast_time(v_b)
        h_b = self.calculate_burn_altitude()
        h_coast = self.calculate_coast_altitude(v_b)
        h_max = self.calculate_max_altitude()
        t_descent = self.calculate_descent_time(h_max)
        t_burn = self.calculate_burn_time()
        t_ascent = self.calculate_total_ascent_time(t_coast)
        t_total = self.calculate_total_flight_time(t_coast, t_descent)
        
        # Calculate accelerations
        a0 = self.calculate_initial_acceleration()
        a_max = self.calculate_max_acceleration()
        a_avg = self.calculate_average_acceleration()
        
        # Calculate thrust
        F_avg = self.calculate_average_thrust()
        
        # Calculate terminal velocity
        v_terminal = self.calculate_terminal_velocity()

        calculation_steps = self.build_calculation_steps()
        
        return {
            'rocket_name': self.rocket.name,
            'motor_name': self.motor.name,
            
            # Mass data (in grams for readability)
            'initial_mass_g': self.m0_kg * 1000,
            'rocket_mass_g': self.mr_kg * 1000,
            'motor_mass_g': self.mm_kg * 1000,
            'propellant_mass_g': self.mp_kg * 1000,
            'burnout_mass_g': self.mb_kg * 1000,
            
            # Thrust data
            'average_thrust_N': F_avg,
            'motor_max_thrust_N': self.motor.max_thrust,
            
            # Acceleration data
            'initial_acceleration_ms2': a0,
            'max_acceleration_ms2': a_max,
            'average_acceleration_ms2': a_avg,
            
            # Velocity data
            'velocity_at_burnout_ms': v_b,
            'max_velocity_ms': v_b,
            'terminal_velocity_ms': v_terminal,
            
            # Altitude data
            'altitude_at_burnout_m': h_b,
            'altitude_during_coast_m': h_coast,
            'max_altitude_m': h_max,
            
            # Flight time segments
            'burn_time_s': t_burn,
            'coast_to_apogee_s': t_coast,
            'descent_time_s': t_descent,
            'total_ascent_time_s': t_ascent,
            'total_flight_time_s': t_total,
            
            # Motor ejection data
            'time_delay_s': self.motor.time_delay,

            # Detailed calculation trace
            'calculation_steps': calculation_steps,
        }


def print_results(results: dict) -> None:
    """Print flight simulation results in readable format"""
    print("\n" + "="*70)
    print(f"MODEL ROCKET FLIGHT SIMULATION")
    print(f"Rocket: {results['rocket_name']} + Motor: {results['motor_name']}")
    print("="*70)

    print("\nSTEP-BY-STEP CALCULATIONS")
    print("-"*70)
    for index, (label, detail) in enumerate(results.get('calculation_steps', []), start=1):
        print(f"{index:>2}. {label}: {detail}")
    
    print(f"\n{'MASS DATA':^70}")
    print("-"*70)
    print(f"  Rocket Dry Mass:           {results['rocket_mass_g']:>8.2f} g")
    print(f"  Motor Total Mass:          {results['motor_mass_g']:>8.2f} g")
    print(f"  Propellant Mass:           {results['propellant_mass_g']:>8.2f} g")
    print(f"  Initial Total Mass (m0):   {results['initial_mass_g']:>8.2f} g")
    print(f"  Burnout Mass (mb):         {results['burnout_mass_g']:>8.2f} g")
    
    print(f"\n{'THRUST PROFILE':^70}")
    print("-"*70)
    print(f"  Motor Max Thrust:          {results['motor_max_thrust_N']:>8.2f} N")
    print(f"  Average Thrust (F_avg):    {results['average_thrust_N']:>8.2f} N")
    print(f"  Burn Time (tb):            {results['burn_time_s']:>8.2f} s")
    
    print(f"\n{'ACCELERATION':^70}")
    print("-"*70)
    print(f"  Initial Acceleration (a0): {results['initial_acceleration_ms2']:>8.2f} m/s²")
    print(f"  Max Acceleration (a_max):  {results['max_acceleration_ms2']:>8.2f} m/s²")
    print(f"  Average Acceleration:      {results['average_acceleration_ms2']:>8.2f} m/s²")
    
    print(f"\n{'VELOCITY':^70}")
    print("-"*70)
    print(f"  Velocity at Burnout (v_b): {results['velocity_at_burnout_ms']:>8.2f} m/s")
    print(f"  Max Velocity:              {results['max_velocity_ms']:>8.2f} m/s")
    print(f"  Terminal Velocity:         {results['terminal_velocity_ms']:>8.2f} m/s")
    
    print(f"\n{'ALTITUDE':^70}")
    print("-"*70)
    print(f"  Altitude at Burnout (h_b): {results['altitude_at_burnout_m']:>8.2f} m")
    print(f"  Altitude Gain (Coast):     {results['altitude_during_coast_m']:>8.2f} m")
    print(f"  Maximum Altitude (h_max):  {results['max_altitude_m']:>8.2f} m")
    
    print(f"\n{'FLIGHT TIMELINE':^70}")
    print("-"*70)
    print(f"  Burn Time:                 {results['burn_time_s']:>8.2f} s")
    print(f"  Coast to Apogee:           {results['coast_to_apogee_s']:>8.2f} s")
    print(f"  Time Delay (ejection):     {results['time_delay_s']:>8.2f} s")
    print(f"  Descent Time:              {results['descent_time_s']:>8.2f} s")
    print(f"  Total Ascent Time:         {results['total_ascent_time_s']:>8.2f} s")
    print(f"  Total Flight Time:         {results['total_flight_time_s']:>8.2f} s")
    
    print("="*70 + "\n")


# Example usage
if __name__ == "__main__":
    
    tiny_rocket = Rocket(
        name="Tiny Rocket",
        length_mm=254,
        diameter_mm=18,
        mass=18,  # grams
        drag_coefficient=0.65
    )
    tiny_motor = Motor(
        name="1/4A3-3T",
        diameter_mm=13,
        length_mm=44,
        total_impulse=0.625,  # N-sec
        thrust_duration=0.25,  # seconds
        max_thrust=4.90,  # N
        time_delay=3,  # seconds
        max_lift_weight=28,  # grams
        total_mass=6,  # grams
        propellant_mass=2.3  # grams
    )

    medium_rocket = Rocket(
        name="Medium Rocket",
        length_mm=381,
        diameter_mm=24,
        mass=196,  # grams
        drag_coefficient=0.65
    )
    b44 = Motor(
        name="B4-4",
        diameter_mm=18,
        length_mm=70,
        total_impulse=5,  # N-sec
        thrust_duration=1.1,  # seconds
        max_thrust=12.8,  # N
        time_delay=4,  # seconds
        max_lift_weight=99,  # grams
        total_mass=18,  # grams
        propellant_mass=7.6  # grams
    )
    b64 = Motor(
        name="B6-4",
        diameter_mm=18,
        length_mm=70,
        total_impulse=5,  # N-sec
        thrust_duration=0.8,  # seconds
        max_thrust=15.0,  # N
        time_delay=4,  # seconds
        max_lift_weight=113,  # grams
        total_mass=17.9,  # grams
        propellant_mass=6.5  # grams
    )

    big_rocket = Rocket(
        name="Big Rocket",
        length_mm=500,
        diameter_mm=29,
        mass=486.4,  # grams
        drag_coefficient=0.65
    )
    big_motor = Motor(
        name="F15-8",
        diameter_mm=29,
        length_mm=114,
        total_impulse=49.61,  # N-sec
        thrust_duration=3.45,  # seconds
        max_thrust=30,  # N
        time_delay=8,  # seconds
        max_lift_weight=425,  # grams
        total_mass=104.6,  # grams
        propellant_mass=60.0  # grams
    )


    # Run simulation
    # simulator = FlightSimulator(tiny_rocket, tiny_motor)
    # results = simulator.run_simulation()
    # print_results(results)

    simulator = FlightSimulator(big_rocket, big_motor)
    results = simulator.run_simulation()
    print_results(results)