#!/usr/bin/env python3
"""
scripts/gen_firmware_manifest.py

Scans docs/firmware/*/ for per-release directories, each containing
one or more `reticulum-rnode-<board>-<tag>.{zip,hex,uf2}` files, and
emits `docs/firmware/manifest.json` for the webflasher.

Output schema:

  {
    "generated_at": "<ISO-8601 UTC>",
    "releases": [
      {
        "tag": "v0.2.0",
        "prerelease": false,
        "boards": [
          {
            "name":     "Faketec",
            "zip_path": "firmware/v0.2.0/reticulum-rnode-Faketec-v0.2.0.zip",
            "zip_size": 235247,
            "uf2_path": "firmware/v0.2.0/reticulum-rnode-Faketec-v0.2.0.uf2",
            "uf2_size": 365568
          }
        ]
      }
    ]
  }

Releases are sorted newest-first. Boards within a release are sorted
alphabetically.
"""

from __future__ import annotations

import datetime as _dt
import json
import re
import sys
from pathlib import Path


REPO_ROOT     = Path(__file__).resolve().parents[1]
FIRMWARE_ROOT = REPO_ROOT / "docs" / "firmware"
MANIFEST_PATH = FIRMWARE_ROOT / "manifest.json"
# Board names may contain hyphens (e.g. "T-Echo"), so match the board
# greedily and anchor the tag on a version-shaped suffix (vMAJOR.MINOR.PATCH
# with an optional prerelease). A non-version-shaped tail won't be mistaken
# for the tag.
ASSET_RE      = re.compile(
    r"^reticulum-rnode-(?P<board>.+)-"
    r"(?P<tag>v\d+\.\d+\.\d+(?:-[0-9A-Za-z.]+)?)\.(?P<ext>zip|hex|uf2)$"
)


def _pad_version_for_sort(tag: str) -> tuple:
    m = re.match(r"^v(\d+)\.(\d+)\.(\d+)(?:-(.+))?$", tag)
    if not m:
        return (0, 0, 0, "")
    major, minor, patch, pre = m.groups()
    return (int(major), int(minor), int(patch), pre or "~")


def scan_releases(firmware_root: Path) -> list[dict]:
    if not firmware_root.is_dir():
        return []

    releases: list[dict] = []
    for entry in sorted(firmware_root.iterdir()):
        if not entry.is_dir():
            continue
        tag = entry.name
        by_board: dict[str, dict] = {}
        for asset in sorted(entry.iterdir()):
            if not asset.is_file():
                continue
            m = ASSET_RE.match(asset.name)
            if not m:
                continue
            board = m.group("board")
            if m.group("tag") != tag:
                print(
                    f"warn: {asset} tag mismatch (expected {tag}, "
                    f"got {m.group('tag')}) — skipping",
                    file=sys.stderr,
                )
                continue
            rec = by_board.setdefault(board, {"name": board})
            ext = m.group("ext")
            rec[f"{ext}_path"] = f"firmware/{tag}/{asset.name}"
            rec[f"{ext}_size"] = asset.stat().st_size

        if not by_board:
            continue

        releases.append({
            "tag":        tag,
            "prerelease": "-" in tag,
            "boards":     sorted(by_board.values(), key=lambda b: b["name"]),
        })

    releases.sort(key=lambda r: _pad_version_for_sort(r["tag"]), reverse=True)
    return releases


def main() -> int:
    FIRMWARE_ROOT.mkdir(parents=True, exist_ok=True)
    releases = scan_releases(FIRMWARE_ROOT)
    manifest = {
        "generated_at": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "releases":     releases,
    }
    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {MANIFEST_PATH} — {len(releases)} release(s)")
    for r in releases:
        boards = ", ".join(b["name"] for b in r["boards"])
        print(f"  {r['tag']}  ({boards}){'  [prerelease]' if r['prerelease'] else ''}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
