# Code and external assets

The code, procedural demo geometry, authored replay trajectories and documentation
in this repository are released under the root MIT license. The bundled paper
replay stores a transcription of Table I's numeric reference values and authored
reconstruction records; it does not bundle the paper PDF or original trial logs.
See `assets/paper_reference.json` in the package for source and assumptions.

External software retains its own license. Runtime dependencies are installed
separately: NumPy (BSD-3-Clause), optional SciPy (BSD-3-Clause), trimesh (MIT),
and Pillow (HPND). FFmpeg is an external executable; use a build with libx264
support for recording. None of those distributions is copied into this package.

Isaac Sim is installed separately under NVIDIA's license. Each user must accept
its terms. The launcher does not accept that agreement on a user's behalf.

The original UR5e/Robotiq USD assembly, dataset OBJ/PLY meshes, Text2HOI weights,
MANO models and participant-derived outputs are **not distributed** here. A local
asset configuration only points at files you already have permission to use.
The repository's MIT license does not grant rights to these external assets.
Generated resolved JSON, USD, videos and hand meshes can embed their geometry:
check the source asset permissions before redistributing those outputs.

The images in `docs/` illustrate locally rendered release demos. They are not
source meshes, trained weights, or original paper experiment records.
