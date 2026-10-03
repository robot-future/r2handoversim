"""Read-only installation checks; importing Isaac Sim itself would start its EULA flow."""
import importlib.util
from importlib.metadata import version, PackageNotFoundError
import platform
import sys
from . import __version__


def inspect_environment(require_isaac=False, video=False, asset_config=None):
    checks = []
    def add(name, ok, detail, required=True):
        checks.append(dict(name=name, status='pass' if ok else 'fail' if required else 'info', detail=detail))
    add('Python', sys.version_info >= (3, 10), f'{platform.python_version()} ({sys.executable})')
    for module in ['numpy'] + (['scipy', 'trimesh', 'PIL'] if asset_config else []):
        add(module, importlib.util.find_spec(module) is not None,
            f'Import available: {module}' if importlib.util.find_spec(module) else
            "Missing; install r2handoversim[planning,assets] in this Python environment")
    installed = importlib.util.find_spec('isaacsim') is not None
    try:
        isaac_version = version('isaacsim')
    except PackageNotFoundError:
        isaac_version = 'standalone installation (version unavailable)'
    add('Isaac Sim', installed, f'{isaac_version}; runtime/GPU launch is checked by demo' if installed else
        "Not found; run in Isaac Sim 5.0's Python environment or use python.sh scripts/run_isaac.py", require_isaac)
    if video:
        from .video import require_encoder
        import subprocess
        try:
            encoder = require_encoder()
            result = subprocess.run([encoder, '-hide_banner', '-encoders'], capture_output=True, text=True, timeout=15)
            add('H.264 recording', result.returncode == 0 and 'libx264' in result.stdout,
                f'{encoder}: libx264 required for MP4 output')
        except (ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
            add('H.264 recording', False, str(exc))
    if asset_config:
        from .local_assets import configuration, attach
        from pathlib import Path
        from .evaluation import validate_trial
        from .demos import load_demo
        try:
            config = configuration(asset_config)
            names = sorted(p.stem for p in Path(config['object_mesh_root']).glob('*.obj'))
            if not names: raise ValueError('No OBJ files in object_mesh_root')
            for name in names:
                trial = load_demo('bottle'); trial['object_id'] = name
                validate_trial(attach([trial], config)[0])
            add('Local assets', True, f'{len(names)} valid OBJ meshes; robot USD exists. USD references, joint names and pad colliders are checked at launch.')
        except (ValueError, KeyError, TypeError, AttributeError, OSError, ImportError) as exc:
            add('Local assets', False, str(exc))
    return dict(schema_version='handover.environment.v1', version=__version__,
                status='failed' if any(c['status'] == 'fail' for c in checks) else 'passed', checks=checks)
