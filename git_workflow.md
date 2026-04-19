# Git Workflow: Viro Cobot Control System

This document captures the standard Git practices and setup for this repository to ensure high-fidelity version control and safety.

## Repository Setup
- **Main Branch**: `main`
- **Remote**: `origin` (`https://github.com/hazeezadebayo/viro_cobot`)

## Initial Configuration (One-time)
```bash
# Add the remote origin
git remote add origin https://github.com/hazeezadebayo/viro_cobot

# Set the default branch to main
git branch -M main
```

## Daily Workflow
1. **Status Check**: `git status`
2. **Stage Changes**: `git add .` (Respects `.gitignore`)
3. **Commit**: `git commit -m "feat: <description>"` or `git commit -m "fix: <description>"`
4. **Push**: `git push origin main`

## Recovery & Precautions
- **Avoid Force Push**: Only use `git push --force` if you are the only one working on the branch and need to overwrite history (e.g., after a rebase).
- **Discard Local Changes**: `git checkout -- <file>` or `git restore <file>`
- **View History**: `git log --oneline --graph --decorate`

## Ignored Files
The following are explicitly excluded from version control via `.gitignore`:
- `build/`, `install/`, `log/` (ROS outputs)
- `project_report.md`, `output_log.md` (Local operational logs)
- `v0_README.md` (Legacy documentation)
