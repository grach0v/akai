---
name: update-rerun-skills
description: Update the vendored official rerun agent skills (.claude/skills/rerun-*) from rerun's git repo, at the rerun-sdk version the rerun node locks. Use when bumping rerun-sdk, when asked to update, refresh or check the rerun skills, or when a rerun-* skill seems out of date with the SDK.
---

# Update the rerun skills

The `rerun-*` skills in `.claude/skills/` are rerun's official agent skills, vendored byte for byte from
https://github.com/rerun-io/rerun/tree/<tag>/skills by `scripts/sync_rerun_skills.py`. The tag is the
`rerun-sdk` version locked in `nodes/rerun/uv.lock`, so the skills always describe the SDK the node
runs. `.claude/skills/.rerun-upstream.json` records the tag, the commit and which skills came from
upstream; `.agents/skills` links to `.claude/skills` for other agents.

## Rules

1. Never edit a vendored `rerun-*` skill: the next sync overwrites it. akai-specific rerun knowledge
   goes in a skill of our own (like this one), which the sync leaves alone.
2. The skills follow the lock, not upstream `main`. To get newer skills, bump `rerun-sdk` first.
3. Commit the skills together with the `rerun-sdk` bump they belong to.

## Steps

1. If the task is a rerun bump: find the release (`gh release view -R rerun-io/rerun`), set
   `rerun-sdk==<version>` in `nodes/rerun/pyproject.toml` (exact: every rerun minor breaks
   something, and the skills describe one version) and the version in `nodes/rerun/README.md`,
   then `uv lock` and `uv sync` in `nodes/rerun`. Read the release's breaking changes (the migration
   guide at https://rerun.io/docs/changelog/changeset-<major>-<minor>) and check each one against the
   rerun API the node uses (`grep -rhoE "\brr[bd]?\.[A-Za-z_.]+" nodes/rerun/src | sort -u`).
2. See whether the skills are stale (exits 1 if they are, changes nothing):
   `uv run scripts/sync_rerun_skills.py --check`
3. Re-vendor: `uv run scripts/sync_rerun_skills.py`. It shallow-clones only `skills/` at the tag,
   replaces every skill it vendored before, adds new ones, drops ones upstream removed, and rewrites
   the manifest. rerun's own maintenance skills are excluded (`--exclude` lists them; a new
   maintenance skill upstream belongs there too).
4. Report what changed: `git diff --stat -- .claude/skills` and the manifest's `tag` and `skills`.
   Name skills added or dropped, and summarise what changed in the updated ones that matters to akai
   (a renamed API, a new recommended pattern), from `git diff -- .claude/skills/<skill>`.
5. Run `uv run scripts/sync_rerun_skills.py --check` again: it must print that the skills match.
