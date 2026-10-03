"""Encode actual Isaac Sim viewport frames without optional Python packages."""
from pathlib import Path
import shutil
import subprocess
import math


def complete_png(path):
    """Kit's capture future may finish before its asynchronous file writer."""
    try:
        with Path(path).open("rb") as stream:
            stream.seek(-12, 2)
            return stream.read() == b"\x00\x00\x00\x00IEND\xaeB`\x82"
    except (OSError, ValueError):
        return False


def require_encoder():
    executable = shutil.which("ffmpeg")
    if executable is None:
        raise ValueError("MP4 recording requires ffmpeg on PATH; install FFmpeg, then retry --video")
    return executable


def encode_frames(directory, destination, count, dt_s, speed=1.):
    if count < 1 or not math.isfinite(dt_s) or dt_s <= 0 or not math.isfinite(speed) or speed <= 0:
        raise ValueError("Video needs frames, a positive timestep and a positive playback speed")
    directory, destination = Path(directory), Path(destination)
    for i in range(count):
        frame = directory / f"{i:06d}.png"
        if not frame.is_file() or frame.stat().st_size == 0:
            raise RuntimeError(f"Missing recorded viewport frame: {frame}")
    temporary = destination.with_name(destination.stem + ".partial.mp4")
    try:
        subprocess.run([require_encoder(), "-hide_banner", "-loglevel", "error", "-y",
            "-framerate", str(speed/dt_s), "-i", str(directory/"%06d.png"),
            "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2,tpad=start_duration=1:stop_duration=2:stop_mode=clone:start_mode=clone",
            "-r", "30", "-c:v", "libx264", "-preset", "fast", "-crf", "18",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(temporary)],
            check=True, capture_output=True, timeout=180)
        if not temporary.is_file() or temporary.stat().st_size == 0:
            raise RuntimeError("FFmpeg produced no video")
        temporary.replace(destination)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"Video encoding failed: {exc.stderr.decode(errors='replace')[-2000:]}") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("Video encoding timed out after 180 seconds") from exc
    finally:
        temporary.unlink(missing_ok=True)


def review_montage(directory, destination):
    """Caption recorded clips with their own scene IDs and measured outcomes."""
    import json
    import tempfile
    from .verification import verify_output
    root=Path(directory).resolve();destination=Path(destination).resolve()
    verify_output(root)
    rows=json.loads((root/'results.json').read_text())
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='r2-review-') as tmp:
        tmp=Path(tmp);clips=[]
        for i,row in enumerate(rows):
            if 'video' not in row['artifacts']: raise ValueError('Every montage trial needs a recorded video')
            trial=json.loads((root/row['artifacts']['trial']).read_text())
            receiver=trial.get('receiver',{})
            caption=f"REPLAY | {row['trial_id']}\n"
            policy=trial.get('grasp_contract',{}).get('feasibility_width_policy','opening') if 'method_selection' in trial else 'geometry demo'
            sampling=('reference IK' if trial.get('receiver_protocol',{}).get('sampling_bounds',{}).get('require_reference_ik')
                      else 'bounded' if receiver else 'provided hand')
            caption+=f"Receiver: {receiver.get('side','provided')} | seed {receiver.get('seed','n/a')} ({sampling}) | {trial['split']} | {policy} | "
            caption+=f"{row['first_failure']+' failure' if row['first_failure'] else 'success'}\n"
            caption+='  '.join(f'{key}: {"n/a" if value is None else "pass" if value else "fail"}' for key,value in row['metrics'].items())
            textfile=tmp/f'{i}.txt';textfile.write_text(caption)
            clip=tmp/f'{i}.mp4';clips.append(clip)
            subprocess.run([require_encoder(),'-hide_banner','-loglevel','error','-y',
                '-i',str(root/row['artifacts']['video']),'-vf',
                f'drawbox=x=0:y=0:w=iw:h=112:color=black@0.72:t=fill,drawtext=textfile={textfile}:expansion=none:fontcolor=white:fontsize=23:x=18:y=12:line_spacing=8',
                '-an','-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p',str(clip)],check=True,capture_output=True,timeout=180)
        manifest=tmp/'clips.txt';manifest.write_text(''.join(f"file '{clip}'\n" for clip in clips))
        partial=destination.with_suffix('.partial.mp4')
        try:
            subprocess.run([require_encoder(),'-hide_banner','-loglevel','error','-y','-f','concat','-safe','0',
                '-i',str(manifest),'-c','copy','-movflags','+faststart',str(partial)],check=True,capture_output=True,timeout=180)
            partial.replace(destination)
        finally: partial.unlink(missing_ok=True)
    return destination
