# Photogrammetry App (GNC)

> This folder is a **git submodule** — the code lives in its own repo:
> **https://github.com/akvaithi/MATE-ROV-Photogrammetry**

A PyQt6 desktop app that turns an RTSP video stream into a 3D model. It watches the
stream, captures frames that pass quality gates (sharp, novel, well-framed), and hands
the image set to a reconstruction engine chosen for the host platform: Apple's Object
Capture (RealityKit) on macOS, or Epic's RealityScan / AliceVision Meshroom on Windows
and Linux.

Built for the MATE ROV competition so an operator can scan an object underwater (or on
deck) via an IP camera and get a usable mesh in a few minutes. Complements the existing
GNC **Photogrammetry Test Files**.

## Backends
| Backend | When to use | Output |
|---|---|---|
| RealityKit (Apple Object Capture) | macOS 12+, no extra tools | `.usdz` |
| RealityScan (Epic) | Windows, any modern GPU; the planned engine for the competition control PC | `.glb` + `.stl` |
| Meshroom (AliceVision) | Windows / Linux with an NVIDIA CUDA GPU | `.glb` + `.stl` |
| Auto (default) | macOS: RealityKit → RealityScan → Meshroom. Windows/Linux: RealityScan → Meshroom → RealityKit | depends |

COLMAP was removed in June 2026. The full design is in the GNC SDD (GNC-02-01-01-R0),
Section 5.

## Getting the code
This is a submodule, so a normal clone leaves this folder empty. To pull it:

```bash
# fresh clone of the team repo, with submodules
git clone --recurse-submodules https://github.com/ThinkTank-TAMU/TAMU-Oceanus.git

# or, if you already cloned the team repo
git submodule update --init --recursive
```

A prebuilt macOS bundle (Apple Silicon, macOS 12+) is also on the
[releases page](https://github.com/akvaithi/MATE-ROV-Photogrammetry/releases/latest).

## Updating this submodule to the latest app version
```bash
cd "01_Subteams/GNC/Photogrammetry-App"
git pull origin main
cd -
git add "01_Subteams/GNC/Photogrammetry-App"
git commit -m "Update photogrammetry submodule"
git push
```

See the app repo's own README for full usage, architecture, and build instructions.
