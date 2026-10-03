"""Exercise the installed wheel from any directory; no simulator required."""
import json
from importlib.resources import files
from importlib.metadata import version
from pathlib import Path
import subprocess
import sys
import tempfile
from r2handoversim import __version__


def main():
    assert version('r2handoversim') == __version__
    with tempfile.TemporaryDirectory(prefix='r2-release-smoke-') as tmp:
        root = Path(tmp)
        def cli(*args):
            subprocess.run([sys.executable,'-m','r2handoversim',*map(str,args)],cwd=root,check=True)
        cli('doctor')
        cli('evaluate','--object','all','--variant','all','--output',root/'offline')
        rows = json.loads((root/'offline/results.json').read_text())
        assert len(rows) == 12 and len({r['trial_id'] for r in rows}) == 12
        cli('paper-replay','--output',root/'paper')
        cli('replay-trial','--record','S1_m4_0000','--output',root/'trial.json')
        cli('evaluate','--trial',root/'trial.json','--output',root/'reference')
        assert files('r2handoversim').joinpath('assets/paper_replay.jsonl.gz').is_file()
        assert (root/'offline/report.html').is_file()
    print(f'Installed r2handoversim {__version__}: release smoke passed')


if __name__ == '__main__':
    main()
