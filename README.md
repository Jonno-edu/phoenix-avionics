# Phoenix Avionics

Central repository for the Phoenix mission avionics stack. This is a monorepo containing firmware for all rocket subsystems, ground support equipment (GSE), and shared libraries.

## System Overview

Phoenix uses a distributed avionics architecture linked by a high-reliability RS485 bus.

### Subsystems (Nodes)

| Module | Description | Path |
|---|---|---|
| **OBC** | On-Board Computer (Master). Flight state, navigation, and telemetry dispatch. | [/OBC](OBC) |
| **EPS** | Electrical Power System. Battery management, power rail control, and monitoring. | [/EPS](EPS) |
| **GSU** | Ground Support Unit. Ground-side interface for testing and launch operations. | [/GSU](GSU) |
| **TrackingRadio** | Long-range telemetry link and recovery beacon controller. | [/TrackingRadio](TrackingRadio) |
| **HILNode** | Hardware-In-The-Loop simulation bridge. | [/HILNode](HILNode) |

### Shared Libraries

| Library | Description |
|---|---|
| `esl-comms` | Shared RS485 framing and protocol engine. |
| `esl-math` | Fixed-point math, DCM, and quaternion utilities. |
| `phoenix-icd` | (Planned) Unified Interface Control Document for across-the-bus communication. |

---

## Development Workflow & Git

This repository follows a single-trunk branching model to ensure all subsystems stay in sync.

### Branching Convention

Do not create long-lived subsystem branches (e.g. `OBC/master`). Instead, use short-lived feature branches scoped by subsystem and type:

`<type>/<subsystem>/<short-description>`

| Branch Category | Example |
|---|---|
| New Feature | `feat/obc/rs485-rx-task` |
| Bug Fix | `fix/gsu/gui-crash` |
| Shared/ICD | `chore/shared/update-icd` |
| Refactor | `refactor/eps/adc-driver` |

### The Workflow

1.  **Branch off master:** Always start from the latest `master`.
    ```bash
    git checkout master
    git pull origin master
    git checkout -b feat/obc/my-feature
    ```
2.  **Atomic Commits:** If a protocol change affects both the OBC and the GSU, **update both in the same commit/branch**. This guarantees the system always integrates correctly.
3.  **Merge to master:** Once verified on hardware, merge back to `master`. Treat `master` as the "Flight Ready" integration branch.
4.  **Clean up:** Delete feature branches after merging.

---

## Getting Started

Each subsystem contains its own specific instructions for building and flashing in its respective directory.

1.  **OBC Setup:** See [OBC/README.md](OBC/README.md)
2.  **Environment:** Ensure you have the `arm-none-eabi-gcc` toolchain and `cmake` installed.

## License

MIT — see [LICENSE](LICENSE).
