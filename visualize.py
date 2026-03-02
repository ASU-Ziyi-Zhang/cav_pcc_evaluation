"""
Simple visualization script for exported time-space CSVs.

This script loads a compressed time-space CSV (produced by
`export_time_space_csv`) and produces a time–space scatter plot where
points are colored by vehicle speed. It sets a default serif font
(prefers Times New Roman), configures plot sizes and labels, and
saves the figure to the scenario's `excel_file` folder before
displaying it.

Inputs:
 - A time-space CSV at the path hardcoded near the top of the file
	 (modify `pd.read_csv(...)` to point to other scenarios).

Outputs:
 - A PNG file written to the scenario `excel_file` folder.
 - An interactive matplotlib window (when run in GUI environments).

Usage:
 - Run `python visualize.py` (ensure the CSV path exists).
"""

import os
import re
import glob
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import font_manager


preferred_fonts = [
	"Times New Roman",            # Windows/macOS if installed
	"Nimbus Roman No9 L",         # Common on many Linux distros (URW)
	"Liberation Serif",           # Widespread replacement
	"DejaVu Serif",               # Always available in many envs
	"FreeSerif",
	"Times"
]
available_fonts = {f.name for f in font_manager.fontManager.ttflist}
chosen_font = next((f for f in preferred_fonts if f in available_fonts), None)
if chosen_font is None:
	# Fallback to generic serif without breaking execution
	chosen_font = "serif"
	print("Warning: Preferred fonts not found. Falling back to 'serif'. "
		  "To use Times New Roman, install a TNR-compatible font package.")
mpl.rcParams["font.family"] = chosen_font
# Make axis tick labels larger; axis labels slightly smaller than ticks/colorbar as requested
mpl.rcParams["xtick.labelsize"] = 14
mpl.rcParams["ytick.labelsize"] = 14
mpl.rcParams["axes.labelsize"] = 14
mpl.rcParams["axes.titlesize"] = 14

base_dir = os.path.join("sumo_scenarios", "i24", "excel_file")


def find_input_file(entry_path):
	# If it's a directory, prefer a file containing 'timespace' else any csv(.gz)
	if os.path.isdir(entry_path):
		candidates = []
		for ext in ("*.csv.gz", "*.csv"):
			candidates.extend(glob.glob(os.path.join(entry_path, ext)))
		if not candidates:
			return None
		timespace = [p for p in candidates if 'timespace' in os.path.basename(p).lower()]
		return timespace[0] if timespace else candidates[0]
	# If it's a file and a csv, use it
	if os.path.isfile(entry_path) and entry_path.endswith(('.csv', '.csv.gz')):
		return entry_path
	# Try to find csv files with the same base name in parent dir
	base = os.path.splitext(entry_path)[0]
	for ext in ('.csv.gz', '.csv'):
		p = base + ext
		if os.path.isfile(p):
			return p
	return None


def plot_timespace(df, out_path):
	for c in ('time', 'veh_id', 'pos', 'speed'):
		if c not in df.columns:
			print(f"Missing column: {c}")
	plt.figure(figsize=(10, 6))
	plt.scatter(df["time"], df["pos"], c=df.get("speed", None), cmap="viridis", s=1, alpha=0.7)
	cbar = plt.colorbar() if 'speed' in df.columns else None
	if cbar is not None:
		cbar.ax.tick_params(labelsize=14)
		cbar.set_label("Speed (m/s)", fontsize=16)
	plt.xlabel("Time (s)", fontsize=16)
	plt.ylabel("Position (m)", fontsize=16)
	plt.xticks(fontsize=14)
	plt.yticks(fontsize=14)
	plt.tight_layout()
	plt.savefig(out_path, dpi=300, bbox_inches="tight")
	plt.close()


if __name__ == "__main__":
	file_pattern = re.compile(r'i24_cav(\d+)_mosharafian2022_timespace.*', re.IGNORECASE)

	files = sorted(os.listdir(base_dir))
	matches = []
	for f in files:
		m = file_pattern.fullmatch(f)
		if m:
			matches.append((f, m.group(1)))

	if not matches:
		print(f"No matching timespace files in {base_dir}")
	else:
		for fname, num in matches:
			in_file = os.path.join(base_dir, fname)
			out_name = f"i24_cav{num}_mosharafian2022.png"
			out_path = os.path.join(base_dir, out_name)
			try:
				df = pd.read_csv(in_file, compression='infer')
			except Exception as e:
				print(f"Failed to read {in_file}: {e}")
				continue
			print(f"Plotting {in_file} -> {out_path}")
			try:
				plot_timespace(df, out_path)
			except Exception as e:
				print(f"Failed to plot {in_file}: {e}")
		print("Done.")

