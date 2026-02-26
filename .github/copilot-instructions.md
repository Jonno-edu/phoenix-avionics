# GitHub Copilot Instructions - Phoenix Avionics

You are the development assistant for the Phoenix Avionics project. Always adhere to the following workflow and architectural rules.

---

## Git Workflow

### The Golden Rule
`master` is the **Flight Ready** branch. Code only lands on `master` once it is verified on hardware.

### Full Feature Workflow

#### 1. Start from a clean master
```bash
git checkout master
git pull origin master
git checkout -b feat/obc/my-feature
```

#### 2. Develop on your branch
Make as many incremental commits as needed. Keep commits scoped and logical.

#### 3. Clean up your branch before merging (Internal Squash)
Before rebasing or merging, collapse your messy development commits into 1-2 logical, meaningful commits using interactive rebase:
```bash
git rebase -i HEAD~<number-of-commits>
# Change 'pick' to 'squash' (or 's') for all commits except the first
```

#### 4. Rebase onto master (NOT merge master into branch)
Once your branch is clean, replay it on top of the latest master:
```bash
git fetch origin
git rebase origin/master
# Resolve any conflicts once, then:
git rebase --continue
```
> **Why rebase and not merge?** Rebasing keeps the branch as a straight, logical line of commits on top of master. Reviewers see only your changes. Conflicts are resolved once, on your clean commits, not repeatedly across messy incremental ones.

#### 5. Squash and Merge into master
```bash
git checkout master
git pull origin master
git merge --squash feat/obc/my-feature
git commit -m "feat(obc): clear summary of the entire feature"
git push origin master
```
**Always use `--squash`.** This condenses all branch commits into a single atomic commit on master, keeping a perfectly linear, bisect-friendly history. Never use `--no-ff` (creates tangled merge webs) or plain fast-forward (loses context).

The commit message should be a clean, descriptive summary — not a list of WIP commits.

#### 6. Clean up
```bash
git branch -d feat/obc/my-feature
git push origin --delete feat/obc/my-feature
```

---

## Atomic Commits
If a protocol change (e.g., ICD update, RS485 message format) affects multiple subsystems (OBC, GSU, EPS), update **all affected subsystems in the same branch/commit**. Never leave the system in a partially-updated state on master.

---

## Branching Convention
`<type>/<subsystem>/<short-description>`

| Type | Example |
|---|---|
| Feature | `feat/obc/rs485-rx-task` |
| Bug Fix | `fix/gsu/gui-crash` |
| Shared/ICD | `chore/shared/update-icd` |
| Refactor | `refactor/eps/adc-driver` |

---

## nORB Architecture
- **Do not** manually edit `topics.h` or add queue initialization to `norb.c`.
- Add a `.msg` file to `OBC/src/norb/msg/` — the CMake build will auto-generate the headers.
- Codegen tool: `OBC/tools/generate_norb_topics.py`.

---

## Code Style
- C firmware: depth-1 FreeRTOS queues for nORB topics, clear HAL/logic separation.
- Use project logging macros from `logging.h`.
