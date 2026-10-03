# Showcase recordings

All clips are viewport recordings from Isaac Sim using the original local
UR5e/Robotiq USD, object meshes and visible MANO receiving hands. The lab preset
uses five area lights; final recordings use path tracing.

| Media | Scene | Presentation |
|---|---|---|
| `handover.gif` / `handover-natural.mp4` | Screwdriver, right receiver, FS; all five checks passed | Synchronized workspace and right-oblique views |
| `receivers.gif` / `receivers-natural.mp4` | Can and screwdriver, left/right receivers, A2, seed 41 | Four full-arm views at 0.5× playback; completed clips hold their final frame |
| `cameras.jpg` / `multiview-natural.mp4` | Same FS screwdriver trajectory as the hero | Overview, left, right and elevated cameras |

[Recording metadata](recordings.json) lists the evaluated montage outcomes.
These scenes demonstrate the workflow; paper-reference aggregate records are
available through the separate `paper-replay` command.

The hero uses a bent-elbow IK pose (55.7° at the endpoint) with 6.3° terminal
wrist roll. The gallery scenes are also selected for posture review within the
same configured receiver bounds.

The GIFs provide inline GitHub previews. Their image links open H.264 MP4
attachments on the benchmark release. Source models and scene inputs are
configured locally under the [external asset terms](../../THIRD_PARTY.md).

To create your own clips, follow [quickstart](../quickstart.md), then replay a
resolved scene with the [camera presets](../rendering.md). `verify-output`
checks the run before `review-video` adds result captions.

## Receiver samples in these clips

Use the configured outward bounds with `receiver-scenes --samples 24 --seed 41`.
The hero is screwdriver / sample 5 / right / FS. The gallery uses A2 for can
samples 10 (left) and 5 (right), and screwdriver samples 0 (left) and 21 (right).
Each scene is passed to method selection before original-collider planning.

Camera replays load the resolved scene. All four hero views have identical
numeric trajectories and evaluated metrics; gallery overview and detail views
use the same paired records.
