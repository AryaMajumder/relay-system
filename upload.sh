#!/usr/bin/env bash
# upload.sh — push this bundle to a GitHub repo.
#
# Usage:
#   ./upload.sh <remote-url>
#
# Examples:
#   ./upload.sh git@github.com:youruser/relay-system.git
#   ./upload.sh https://github.com/youruser/relay-system.git
#
# What this does:
#   1. Initialises a git repo in this directory if one does not exist.
#   2. Stages every non-gitignored file.
#   3. Creates an initial commit (or a follow-up commit if the tree changed).
#   4. Adds the remote as `origin` (or updates it if present).
#   5. Pushes to `main`.
#
# Prerequisites:
#   - `git` installed and configured (user.name, user.email).
#   - An **empty** GitHub repo already created at <remote-url>. Create it
#     via the browser or `gh repo create <name> --public --confirm` if you
#     have gh installed.
#   - Push credentials working — SSH key if using git@github.com:...,
#     or a personal access token cached via `git credential-manager` /
#     `git config credential.helper store` for https://...

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <remote-url>" >&2
  echo "Example: $0 git@github.com:youruser/relay-system.git" >&2
  exit 1
fi

REMOTE="$1"
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

# --- git identity sanity check ---
if ! git config --global user.email >/dev/null 2>&1 && ! git config user.email >/dev/null 2>&1; then
  echo "→ git user.email is not set. Configure it first:" >&2
  echo "    git config --global user.email 'you@example.com'" >&2
  echo "    git config --global user.name  'Your Name'" >&2
  exit 1
fi

# --- init if needed ---
if [ ! -d .git ]; then
  echo "→ Initialising git repo"
  git init -b main
fi

# --- current branch → main ---
CURRENT="$(git symbolic-ref --short HEAD 2>/dev/null || echo main)"
if [ "$CURRENT" != "main" ]; then
  echo "→ Renaming current branch to main"
  git branch -M main
fi

# --- stage everything (respects .gitignore) ---
echo "→ Staging files"
git add -A

# --- commit if the tree changed ---
if git diff --cached --quiet; then
  echo "→ No staged changes. Skipping commit."
else
  echo "→ Committing"
  git commit -m "Relay system bundle"
fi

# --- remote ---
if git remote get-url origin >/dev/null 2>&1; then
  echo "→ Updating origin to $REMOTE"
  git remote set-url origin "$REMOTE"
else
  echo "→ Adding origin $REMOTE"
  git remote add origin "$REMOTE"
fi

# --- push ---
echo "→ Pushing to origin/main"
git push -u origin main

echo
echo "── Upload complete ──"
echo "Repo: $REMOTE"
