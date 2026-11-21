"""
package_submission.py - Submission Bundle Packager for XR Hackathon Track 3

Builds the official submission archive matching Slide 17 specifications:
  <team_name>.zip
  ├── estimated_trajectory.txt   # final generated trajectory file
  ├── metadata.json              # optional: runtime perf, compute specs
  └── src/                       # complete source for reproducibility
      ├── main.py                # entrypoint
      └── README.md              # instructions
"""

import os
import sys
import zipfile
import argparse
import shutil


def package_submission(team_name: str = "PathFinder_Track3", output_dir: str = "."):
    zip_filename = f"{team_name}.zip"
    zip_path = os.path.join(output_dir, zip_filename)

    # Required files
    required_files = [
        ("estimated_trajectory.txt", "estimated_trajectory.txt"),
        ("metadata.json", "metadata.json"),
        ("main.py", "src/main.py"),
        ("README.md", "src/README.md"),
        ("degrade_imu_default.py", "src/degrade_imu.py"),
        ("requirements.txt", "src/requirements.txt")
    ]

    # Verify presence
    for src_file, _ in required_files:
        if not os.path.exists(src_file):
            raise FileNotFoundError(f"Missing required deliverable for packaging: {src_file}")

    # Build zip
    print(f"[package_submission] Building official submission archive: {zip_path}")
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for src_file, arc_name in required_files:
            zf.write(src_file, arcname=arc_name)
            print(f"  + Added: {arc_name} (from {src_file})")

    # Verify zip integrity
    print(f"\n[Verification] Checking {zip_filename} archive contents:")
    with zipfile.ZipFile(zip_path, "r") as zf:
        for info in zf.infolist():
            print(f"  - {info.filename:30} ({info.file_size:,} bytes)")

    print(f"\n[SUCCESS] Successfully packaged submission bundle: {zip_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Package Track 3 submission bundle.")
    parser.add_argument("--team-name", default="CollegeName_TeamName",
                        help="Team name following CollegeName_TeamName rule.")
    args = parser.parse_args()
    package_submission(team_name=args.team_name)
