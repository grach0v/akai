"""Vendor rerun's agent skills at the rerun version the rerun node uses.

Rerun publishes skills that teach a coding agent its SDK (data model, URDF, blueprints,
dataset conversion, ...) in https://github.com/rerun-io/rerun/tree/main/skills. This
copies them into `.claude/skills/` (Claude Code; `.agents/skills` links to it for Codex,
Cursor and others), taken from the release tag of the `rerun-sdk` version locked in
`nodes/rerun/uv.lock`, so the skills always describe the SDK the node runs.

To update: bump `rerun-sdk` in `nodes/rerun/pyproject.toml`, `uv lock` there, then

    uv run scripts/sync_rerun_skills.py            # re-vendor the skills
    uv run scripts/sync_rerun_skills.py --check    # exit 1 if the vendored skills are stale

The vendored skills are upstream's, byte for byte: never edit them here (a sync would
overwrite the edit); akai-specific rerun notes belong in a skill of our own. Which skills
came from upstream, and from which tag and commit, is recorded in
`.claude/skills/.rerun-upstream.json`; a sync replaces exactly those, so our own skills in
the same directory are left alone, and a skill upstream dropped is removed.
"""

from __future__ import annotations

import argparse
import filecmp
import json
import shutil
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def locked_rerun_version(lock: Path) -> str:
    packages = tomllib.loads(lock.read_text())["package"]
    (version,) = [p["version"] for p in packages if p["name"] == "rerun-sdk"]
    return version


def fetch_skills(tag: str, into: Path) -> tuple[Path, str]:
    """Upstream's `skills/` directory at `tag` (a sparse, blob-less shallow clone) and its commit."""
    url = "https://github.com/rerun-io/rerun"
    subprocess.run(
        [
            "git",
            "-c",
            "advice.detachedHead=false",
            "clone",
            "--quiet",
            "--depth",
            "1",
            "--filter=blob:none",
            "--sparse",
            "--branch",
            tag,
            url,
            str(into),
        ],
        check=True,
    )
    subprocess.run(["git", "-C", str(into), "sparse-checkout", "set", "skills"], check=True)
    commit = subprocess.run(
        ["git", "-C", str(into), "rev-parse", "HEAD"], check=True, capture_output=True, text=True
    ).stdout.strip()
    return into / "skills", commit


def upstream_skills(skills: Path, exclude: list[str]) -> list[str]:
    """Skill directories (those with a SKILL.md), minus the excluded ones."""
    return sorted(d.name for d in skills.iterdir() if (d / "SKILL.md").is_file() and d.name not in exclude)


def same_tree(a: Path, b: Path) -> bool:
    cmp = filecmp.dircmp(a, b)
    if cmp.left_only or cmp.right_only or cmp.funny_files:
        return False
    _, mismatch, errors = filecmp.cmpfiles(a, b, cmp.common_files, shallow=False)
    return not mismatch and not errors and all(same_tree(a / d, b / d) for d in cmp.common_dirs)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--check", action="store_true", help="change nothing; exit 1 if the vendored skills are stale"
    )
    parser.add_argument(
        "--exclude",
        nargs="*",
        default=["assemble-changelog", "investigate-puffin"],  # rerun's own maintenance skills
        help="upstream skills not to vendor",
    )
    parser.add_argument(
        "--lock", type=Path, default=REPO / "nodes/rerun/uv.lock", help="lock file with rerun-sdk"
    )
    parser.add_argument("--dest", type=Path, default=REPO / ".claude/skills", help="where the skills go")
    args = parser.parse_args()

    tag = locked_rerun_version(args.lock)
    manifest_path = args.dest / ".rerun-upstream.json"
    vendored = json.loads(manifest_path.read_text())["skills"] if manifest_path.is_file() else []

    with tempfile.TemporaryDirectory() as tmp:
        skills, commit = fetch_skills(tag, Path(tmp) / "rerun")
        names = upstream_skills(skills, args.exclude)

        if args.check:
            stale = sorted(set(vendored) - set(names)) + [
                n for n in names if not (args.dest / n).is_dir() or not same_tree(skills / n, args.dest / n)
            ]
            if stale:
                print(f"rerun skills differ from rerun {tag} ({commit[:10]}): {', '.join(stale)}")
                print("run: uv run scripts/sync_rerun_skills.py")
                return 1
            print(f"rerun skills match rerun {tag} ({commit[:10]})")
            return 0

        args.dest.mkdir(parents=True, exist_ok=True)
        for name in sorted(set(vendored) | set(names)):  # drop what we vendored before, dropped upstream too
            shutil.rmtree(args.dest / name, ignore_errors=True)
        for name in names:
            shutil.copytree(skills / name, args.dest / name)
        manifest = {
            "source": f"https://github.com/rerun-io/rerun/tree/{tag}/skills",
            "tag": tag,
            "commit": commit,
            "skills": names,
            "note": "Vendored by scripts/sync_rerun_skills.py; do not edit these skills here.",
        }
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

    agents = REPO / ".agents/skills"  # the path Codex, Cursor and other agents read
    if not agents.exists():
        agents.parent.mkdir(exist_ok=True)
        agents.symlink_to(Path("..") / args.dest.relative_to(REPO))
    print(f"vendored {len(names)} rerun skills from {tag} ({commit[:10]}) into {args.dest.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
