"""Run with Isaac Sim's python.sh without an editable install."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from r2handoversim.cli import main

main()
