"""Internal subprocess entry point; users should run the public CLI."""
import json
from pathlib import Path
import sys


def main():
    job = json.loads(Path(sys.argv[1]).read_text())
    try:
        from .isaac_backend import replay
        replay(**job)
    except BaseException as exc:
        manifest = Path(job["output"]) / "run.json"
        state = json.loads(manifest.read_text())
        state.update(status="failed", error=str(exc))
        manifest.write_text(json.dumps(state, indent=2))
        raise


if __name__ == "__main__":
    main()
