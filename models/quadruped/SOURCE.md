# Unitree Go2 — source provenance

Vendored without modification from [Google DeepMind MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/f3b6013629c7b94f8ee93fd977b161fa8350f9bc/unitree_go2).

- Upstream commit: `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`
- Directory: `unitree_go2`
- Retrieved: 2026-10-24
- License: [BSD-3-Clause](LICENSE), retained with the upstream documentation.
- Default scene: [scene.xml](scene.xml), including [go2.xml](go2.xml) and 16 OBJ mesh assets in [assets](assets/).
- Upstream README, changelog, MJX variants and preview images are retained.

The C++ viewer loads this scene directly. It resets to the upstream `home` keyframe and clears actuator `ctrl` to zero before starting the selected controller. No modifications are made to the upstream model, collision geometry, inertias or actuator limits. The SMC adapter computes motor torques in C++; passive mode leaves them zero. Waypoint markers are rendering-only geometry.
