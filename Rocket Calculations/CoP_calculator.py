"""
Center of Pressure calculator for model rockets using Jim Barrowman equations.

This script estimates rocket CoP for a simple configuration:
- Nose cone
- Cylindrical body tube
- One fin set

Inputs are accepted in millimeters and converted to meters internally.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass


@dataclass
class Nose:
	shape: str
	length_m: float


@dataclass
class FinSet:
	count: int
	span_m: float
	root_chord_m: float
	tip_chord_m: float
	sweep_le_m: float
	leading_edge_from_tip_m: float


@dataclass
class Rocket:
	diameter_m: float
	body_length_m: float
	nose: Nose
	fins: FinSet

	@property
	def total_length_m(self) -> float:
		return self.nose.length_m + self.body_length_m


def mm_to_m(value_mm: float) -> float:
	return value_mm / 1000.0


def nose_cp_from_tip(nose: Nose) -> float:
	"""Return nose CP location from nose tip using common Barrowman approximations."""
	shape = nose.shape.strip().lower()

	if shape in {"cone", "conical"}:
		return (2.0 / 3.0) * nose.length_m
	if shape in {"ogive", "tangent_ogive", "tangent-ogive"}:
		return 0.466 * nose.length_m
	if shape in {"parabolic", "parabola"}:
		return 0.5 * nose.length_m
	if shape in {"elliptical", "ellipse"}:
		return (1.0 / 3.0) * nose.length_m

	raise ValueError(
		"Unsupported nose type. Use one of: conical, ogive, parabolic, elliptical."
	)


def fin_cna(fin: FinSet, diameter_m: float) -> float:
	"""Barrowman normal-force slope contribution of a fin set."""
	if fin.count <= 0:
		return 0.0

	if diameter_m <= 0 or fin.span_m <= 0:
		raise ValueError("Diameter and fin span must be > 0.")

	chord_sum = fin.root_chord_m + fin.tip_chord_m
	if chord_sum <= 0:
		raise ValueError("Root chord + tip chord must be > 0.")

	mid_chord_sweep = fin.sweep_le_m + 0.5 * (fin.tip_chord_m - fin.root_chord_m)
	fin_mid_chord_line = math.sqrt(fin.span_m**2 + mid_chord_sweep**2)

	interference_factor = 1.0 + (diameter_m / (2.0 * fin.span_m + diameter_m))
	slope_num = 4.0 * fin.count * (fin.span_m / diameter_m) ** 2
	slope_den = 1.0 + math.sqrt(1.0 + (2.0 * fin_mid_chord_line / chord_sum) ** 2)

	return interference_factor * (slope_num / slope_den)


def fin_cp_from_tip(fin: FinSet) -> float:
	"""Barrowman fin-set CP location from nose tip."""
	chord_sum = fin.root_chord_m + fin.tip_chord_m
	if chord_sum <= 0:
		raise ValueError("Root chord + tip chord must be > 0.")

	local_cp = (
		(fin.sweep_le_m / 3.0) * ((fin.root_chord_m + 2.0 * fin.tip_chord_m) / chord_sum)
		+ (1.0 / 6.0)
		* (fin.root_chord_m + fin.tip_chord_m - (fin.root_chord_m * fin.tip_chord_m / chord_sum))
	)
	return fin.leading_edge_from_tip_m + local_cp


def calculate_cop(rocket: Rocket) -> dict[str, float]:
	"""Compute CoP using weighted Barrowman component contributions."""
	nose_cna = 2.0
	nose_x = nose_cp_from_tip(rocket.nose)

	fins_cna = fin_cna(rocket.fins, rocket.diameter_m)
	fins_x = fin_cp_from_tip(rocket.fins)

	total_cna = nose_cna + fins_cna
	if total_cna <= 0:
		raise ValueError("Total normal-force slope is zero; cannot compute CoP.")

	x_cp = ((nose_cna * nose_x) + (fins_cna * fins_x)) / total_cna

	return {
		"nose_cna": nose_cna,
		"nose_x_m": nose_x,
		"fins_cna": fins_cna,
		"fins_x_m": fins_x,
		"total_cna": total_cna,
		"x_cp_m": x_cp,
		"x_cp_mm": x_cp * 1000.0,
	}


def parse_fin_type(fin_type: str, root_chord_m: float, tip_chord_m: float) -> tuple[float, float]:
	"""Normalize fin geometry based on requested fin planform type."""
	key = fin_type.strip().lower()
	if key in {"trapezoid", "trapezoidal"}:
		return root_chord_m, tip_chord_m
	if key in {"triangular", "delta"}:
		return root_chord_m, 0.0
	if key in {"rectangular", "rectangle"}:
		return root_chord_m, root_chord_m
	raise ValueError("Unsupported fin type. Use: trapezoidal, triangular, or rectangular.")


def prompt_float(prompt: str, default: float | None = None) -> float:
	while True:
		suffix = ""
		if default is not None:
			suffix = f" [{default}]"
		raw = input(f"{prompt}{suffix}: ").strip()
		if not raw and default is not None:
			return float(default)
		try:
			value = float(raw)
			return value
		except ValueError:
			print("Please enter a numeric value.")


def prompt_int(prompt: str, default: int | None = None) -> int:
	while True:
		suffix = ""
		if default is not None:
			suffix = f" [{default}]"
		raw = input(f"{prompt}{suffix}: ").strip()
		if not raw and default is not None:
			return int(default)
		try:
			value = int(raw)
			return value
		except ValueError:
			print("Please enter an integer value.")


def build_rocket_from_cli(args: argparse.Namespace) -> Rocket:
	root_chord_m = mm_to_m(args.fin_root_chord_mm)
	tip_chord_m = mm_to_m(args.fin_tip_chord_mm)
	root_chord_m, tip_chord_m = parse_fin_type(args.fin_type, root_chord_m, tip_chord_m)

	if args.fin_le_from_tip_mm is None:
		fin_le_from_tip_m = mm_to_m(args.nose_length_mm + args.body_length_mm - args.fin_root_chord_mm)
	else:
		fin_le_from_tip_m = mm_to_m(args.fin_le_from_tip_mm)

	nose = Nose(shape=args.nose_type, length_m=mm_to_m(args.nose_length_mm))
	fins = FinSet(
		count=args.fin_count,
		span_m=mm_to_m(args.fin_span_mm),
		root_chord_m=root_chord_m,
		tip_chord_m=tip_chord_m,
		sweep_le_m=mm_to_m(args.fin_sweep_le_mm),
		leading_edge_from_tip_m=fin_le_from_tip_m,
	)
	return Rocket(
		diameter_m=mm_to_m(args.diameter_mm),
		body_length_m=mm_to_m(args.body_length_mm),
		nose=nose,
		fins=fins,
	)


def build_rocket_interactive() -> Rocket:
	print("Barrowman Center of Pressure Calculator")
	print("Enter all dimensions in mm.\n")

	diameter_mm = prompt_float("Body diameter (mm)")
	total_length_mm = prompt_float("Total rocket length (mm)")
	nose_length_mm = prompt_float("Nose length (mm)", default=0.2 * total_length_mm)
	body_length_mm = total_length_mm - nose_length_mm
	if body_length_mm <= 0:
		raise ValueError("Nose length must be less than total rocket length.")

	nose_type = input("Nose type [conical]: ").strip() or "conical"

	fin_type = input("Fin type (trapezoidal/triangular/rectangular) [trapezoidal]: ").strip() or "trapezoidal"
	fin_count = prompt_int("Number of fins", default=3)
	fin_span_mm = prompt_float("Fin span (mm)")
	fin_root_chord_mm = prompt_float("Fin root chord (mm)")
	fin_tip_chord_mm = prompt_float("Fin tip chord (mm)", default=0.5 * fin_root_chord_mm)
	fin_sweep_le_mm = prompt_float("Leading-edge sweep distance (mm)", default=0.0)

	fin_default = total_length_mm - fin_root_chord_mm
	fin_le_from_tip_mm = prompt_float(
		"Fin root leading edge from nose tip (mm)",
		default=fin_default,
	)

	root_chord_m = mm_to_m(fin_root_chord_mm)
	tip_chord_m = mm_to_m(fin_tip_chord_mm)
	root_chord_m, tip_chord_m = parse_fin_type(fin_type, root_chord_m, tip_chord_m)

	nose = Nose(shape=nose_type, length_m=mm_to_m(nose_length_mm))
	fins = FinSet(
		count=fin_count,
		span_m=mm_to_m(fin_span_mm),
		root_chord_m=root_chord_m,
		tip_chord_m=tip_chord_m,
		sweep_le_m=mm_to_m(fin_sweep_le_mm),
		leading_edge_from_tip_m=mm_to_m(fin_le_from_tip_mm),
	)

	return Rocket(
		diameter_m=mm_to_m(diameter_mm),
		body_length_m=mm_to_m(body_length_mm),
		nose=nose,
		fins=fins,
	)


def create_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(
		description="Calculate center of pressure using Barrowman equations.",
	)
	parser.add_argument("--interactive", action="store_true", help="Prompt for values interactively.")

	parser.add_argument("--diameter-mm", type=float, help="Body diameter in mm.")
	parser.add_argument("--nose-length-mm", type=float, help="Nose length in mm.")
	parser.add_argument("--body-length-mm", type=float, help="Body tube length in mm.")
	parser.add_argument("--nose-type", type=str, default="conical", help="Nose type: conical, ogive, parabolic, elliptical.")

	parser.add_argument("--fin-type", type=str, default="trapezoidal", help="Fin type: trapezoidal, triangular, rectangular.")
	parser.add_argument("--fin-count", type=int, help="Number of fins.")
	parser.add_argument("--fin-span-mm", type=float, help="Fin span in mm.")
	parser.add_argument("--fin-root-chord-mm", type=float, help="Fin root chord in mm.")
	parser.add_argument("--fin-tip-chord-mm", type=float, default=0.0, help="Fin tip chord in mm.")
	parser.add_argument("--fin-sweep-le-mm", type=float, default=0.0, help="Leading-edge sweep distance in mm.")
	parser.add_argument(
		"--fin-le-from-tip-mm",
		type=float,
		default=None,
		help="Distance from nose tip to fin root leading edge in mm. Defaults to tail-mounted.",
	)
	return parser


def validate_cli_args(args: argparse.Namespace) -> None:
	required = [
		"diameter_mm",
		"nose_length_mm",
		"body_length_mm",
		"fin_count",
		"fin_span_mm",
		"fin_root_chord_mm",
	]
	missing = [name for name in required if getattr(args, name) is None]
	if missing:
		joined = ", ".join("--" + name.replace("_", "-") for name in missing)
		raise ValueError(f"Missing required arguments: {joined}")


def print_report(rocket: Rocket, result: dict[str, float]) -> None:
	print("\n=== Barrowman CoP Report ===")
	print(f"Diameter:            {rocket.diameter_m * 1000:.2f} mm")
	print(f"Total length:        {rocket.total_length_m * 1000:.2f} mm")
	print(f"Nose type:           {rocket.nose.shape}")
	print(f"Nose CNa:            {result['nose_cna']:.4f}")
	print(f"Nose CP from tip:    {result['nose_x_m'] * 1000:.2f} mm")
	print(f"Fin set CNa:         {result['fins_cna']:.4f}")
	print(f"Fin set CP from tip: {result['fins_x_m'] * 1000:.2f} mm")
	print(f"Total CNa:           {result['total_cna']:.4f}")
	print(f"\nCenter of Pressure from tip: {result['x_cp_mm']:.2f} mm ({result['x_cp_m']:.4f} m)")


def main() -> None:
	parser = create_parser()
	args = parser.parse_args()

	required_fields = [
		args.diameter_mm,
		args.nose_length_mm,
		args.body_length_mm,
		args.fin_count,
		args.fin_span_mm,
		args.fin_root_chord_mm,
	]

	if args.interactive or not any(value is not None for value in required_fields):
		rocket = build_rocket_interactive()
	else:
		validate_cli_args(args)
		rocket = build_rocket_from_cli(args)

	result = calculate_cop(rocket)
	print_report(rocket, result)


if __name__ == "__main__":
	main()
