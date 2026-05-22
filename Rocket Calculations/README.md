# Rocket Calculations

This folder contains scripts and data for model-rocket calculations and simple flight simulations.

Contents
- `CoP_calculator.py`: Barrowman center-of-pressure (CoP) calculator. Accepts dimensions in millimeters and outputs CoP location and component CNa values.
- `model_rocket_calculator.py`: Flight-simulation framework providing `Motor`, `Rocket`, and `FlightSimulator` classes for estimating acceleration, burnout velocity, apogee, and flight times.
- `FLIGHT7.CSV`, `FLIGHT7_estimated.csv`: Example flight data / exports used for reference.

Requirements
- Python 3.8+ (no external packages required).

Usage

CoP calculator (interactive):

```bash
python "Rocket Calculations/CoP_calculator.py" --interactive
```

CoP calculator (CLI example):

```bash
python "Rocket Calculations/CoP_calculator.py" \
  --diameter-mm 54 \
  --nose-length-mm 150 \
  --body-length-mm 600 \
  --fin-count 3 \
  --fin-span-mm 80 \
  --fin-root-chord-mm 40
```

Flight simulator (import and quick example):

Create a small script or run in the Python REPL:

```python
from model_rocket_calculator import Rocket, Motor, FlightSimulator

rocket = Rocket(name="TestRocket", length_mm=600, diameter_mm=54, mass=300, drag_coefficient=0.5)
motor = Motor(
    name="SampleMotor",
    diameter_mm=18,
    length_mm=70,
    total_impulse=5.0,       # N·s
    thrust_duration=0.8,     # s
    max_thrust=10.0,         # N
    time_delay=3.0,          # s (ejection delay)
    max_lift_weight=300,     # g
    total_mass=20.0,         # g
    propellant_mass=10.0,    # g
)

sim = FlightSimulator(rocket, motor)
print("Estimated max altitude (m):", sim.calculate_max_altitude())
```

Notes
- Inputs in the CoP calculator are expected in millimeters; outputs include both meters and millimeters where appropriate.
- The flight simulator uses simple drag-aware numerical integration and reasonable default air density and gravity constants; treat results as estimates rather than precise predictions.

If you want, I can add an example runner script or unit tests for these modules.
