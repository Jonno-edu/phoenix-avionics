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

3.  **Keep your branch clean:** Before merging, squash your incremental development commits into 1–2 logical commits using interactive rebase.
    ```bash
    git rebase -i HEAD~<number-of-commits>
    # Mark all but the first as 'squash' or 's'
    ```

4.  **Rebase onto master:** Once your branch is clean, replay it on top of the latest `master`. This keeps a straight, linear history and means any conflict is resolved only once.
    ```bash
    git fetch origin
    git rebase origin/master
    # Resolve any conflicts, then:
    git rebase --continue
    ```

5.  **Squash and Merge to master:** Once verified on hardware, squash-merge into `master`. Treat `master` as the "Flight Ready" branch — it must always be a clean, linear, bisect-friendly history.
    ```bash
    git checkout master
    git pull origin master
    git merge --squash feat/obc/my-feature
    git commit -m "feat(obc): clear summary of the entire feature"
    git push origin master
    ```
    > **Never use `--no-ff`.** Merge commits create tangled histories that make `git bisect` and `git revert` painful. Each entry on `master` must represent one complete, working, tested feature.

6.  **Clean up:** Delete the feature branch after merging.
    ```bash
    git branch -d feat/obc/my-feature
    git push origin --delete feat/obc/my-feature
    ```

> **Why rebase instead of merging master into the branch?**
> Rebasing keeps your feature branch as a clean, straight line of logical commits on top of master. When another developer reviews your PR, they see only your changes — not interleaved upstream commits. It also means conflict resolution happens only once, on your final clean commits, rather than repeatedly across many messy incremental ones.
2.  **Environment:** Ensure you have the `arm-none-eabi-gcc` toolchain and `cmake` installed.

## License

MIT — see [LICENSE](LICENSE).
