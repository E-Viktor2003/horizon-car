#!/usr/bin/env python3
"""Package a Horizon native bundle into a deterministic .hnmod, or check one.

A Horizon native bundle is a folder with `horizon-mixin.toml` (plus optional
`[api]` crate, `[[crate]]` crates, helpers, modules and `data/`). The registry
file is a zip with:

    horizon-native.json
    mixins/<mod id>/horizon-mixin.toml
    mixins/<mod id>/...          (.rs, .toml, data/..., README/LICENSE)

Usage:
    python3 package_native.py path/to/bundle
    python3 package_native.py path/to/bundle -o build/my_mod.hnmod
    python3 package_native.py --check build/my_mod.hnmod

Pins are read from `compatibility.json` (the official mods' layout) or from
`horizon-mixin.toml`'s `[compatibility]`. A registry file needs both exact
revisions, and the Horizon pin must be in `horizon-mixin.toml`; pass --mixins
if the bundle does not carry a `compatibility.json`. --horizon only checks an
expected pin, it does not rewrite the manifest.

This script only packs and checks structure. It does not compile or run code,
and it does not sign the file. See references/native-mods.md and
references/publishing.md.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import tomllib
import zipfile
from pathlib import Path, PurePosixPath

PIN = re.compile(r"^[0-9a-f]{40}$")
MOD_ID = re.compile(r"^[a-z0-9_]{1,64}$")
KIND_NAMES = ("README", "LICENSE", "LICENCE", "COPYING", "NOTICE", ".gitignore")
MAX_NATIVE_FILES = 512
MAX_NATIVE_BYTES = 64 << 20
MAX_FILE = 64 << 20


def fail(message: str) -> "NoReturn":
    print(f"error: {message}", file=sys.stderr)
    raise SystemExit(1)


def load_toml(path: Path) -> dict:
    try:
        with path.open("rb") as f:
            return tomllib.load(f)
    except FileNotFoundError:
        fail(f"{path}: no such file")
    except tomllib.TOMLDecodeError as e:
        fail(f"{path}: {e}")


def read_meta(bundle: Path, args) -> dict:
    """The horizon-native.json fields, from the bundle's own files and flags."""
    mixin_path = bundle / "horizon-mixin.toml"
    mixin = load_toml(mixin_path)
    mod = mixin.get("mod", {})
    compatibility = mixin.get("compatibility", {})
    if not isinstance(mod, dict) or not isinstance(compatibility, dict):
        fail(f"{mixin_path}: [mod] and [compatibility] must be tables")

    mod_id = str(mod.get("id", ""))
    version = str(mod.get("version", ""))
    if not MOD_ID.fullmatch(mod_id):
        fail(f"{mixin_path}: [mod] id {mod_id!r} is not [a-z0-9_]{{1,64}}")
    if not version:
        fail(f"{mixin_path}: [mod] version is empty")

    extra = {}
    compat_path = bundle / "compatibility.json"
    if compat_path.is_file():
        try:
            extra = json.loads(compat_path.read_text())
        except json.JSONDecodeError as e:
            fail(f"{compat_path}: {e}")
        if not isinstance(extra, dict):
            fail(f"{compat_path}: must hold a JSON object")

    # A registry file's horizon must be the bundle's own pin: "native manifest
    # and mixin identity/revision differ" is refused by the launcher.
    pinned = compatibility.get("horizon_revision")
    if pinned and args.horizon and pinned != args.horizon:
        fail(f"--horizon {args.horizon} differs from the bundle's pin {pinned}")
    horizon = pinned or extra.get("horizon_revision")
    mixins = args.mixins or extra.get("horizon_mixins_revision")
    game = args.game or extra.get("game") or "26.3"
    hooks = args.hooks if args.hooks is not None else extra.get("hooks")

    if not horizon:
        fail(
            "no Horizon pin: set [compatibility] horizon_revision in "
            "horizon-mixin.toml"
        )
    if not mixins:
        fail(
            "no horizon-mixins pin: put horizon_mixins_revision in "
            "compatibility.json, or pass --mixins <40 hex>"
        )
    for name, value in (("horizon", horizon), ("mixins", mixins)):
        if not PIN.fullmatch(str(value)):
            fail(f"{name} {value!r} is not 40 lowercase hex characters")
    if not pinned:
        fail(
            "horizon-mixin.toml has no [compatibility] horizon_revision; "
            "a .hnmod's horizon must be the bundle's own pin (the launcher refuses "
            "a file whose manifest and mixin revision differ)"
        )
    if hooks is not None:
        if not isinstance(hooks, int) or hooks < 1:
            fail(f"hooks {hooks!r} is not a positive integer")
    if not game:
        fail("game is empty")

    return {
        "schema": 1,
        "id": mod_id,
        "version": version,
        "game": str(game),
        "horizon": str(horizon),
        "mixins": str(mixins),
        **({"hooks": hooks} if hooks is not None else {}),
    }


def allowed(rel: PurePosixPath) -> bool:
    """Whether the bundle reader accepts this path under mixins/<id>/."""
    parts = rel.parts
    if not parts:
        return False
    if rel.suffix in (".rs", ".toml"):
        return True
    # data/ at the bundle root, or the data/ folder of a top-level crate.
    if parts[0] == "data" or (len(parts) >= 2 and parts[1] == "data"):
        return True
    # Licence and readme files in the bundle root or a top-level folder.
    if rel.name.startswith(KIND_NAMES) and len(parts) <= 2:
        return True
    return False


def bundle_files(bundle: Path) -> list[tuple[PurePosixPath, Path]]:
    files = []
    for path in sorted(bundle.rglob("*")):
        if not path.is_file() or path.is_symlink():
            continue
        rel = PurePosixPath(path.relative_to(bundle).as_posix())
        # Local build and VCS folders: a checkout of Horizon next to the mod
        # must never leak into the bundle as hundreds of allowed .rs files.
        if rel.parts[0] in (".git", ".cargo", "target", "node_modules", "dist", "horizon"):
            continue
        if rel in (PurePosixPath("horizon-native.json"), PurePosixPath("horizon-mod.toml")):
            # The first is regenerated; content belongs in an .hmod's top level,
            # not inside a native bundle.
            continue
        if allowed(rel):
            files.append((rel, path))
        else:
            print(f"skipping {rel} (not allowed in an .hnmod)", file=sys.stderr)
    return files


def package(bundle: Path, output: Path | None, args) -> None:
    if not (bundle / "horizon-mixin.toml").is_file():
        fail(f"{bundle}: no horizon-mixin.toml")
    meta = read_meta(bundle, args)
    files = bundle_files(bundle)
    if len(files) + 1 > MAX_NATIVE_FILES:
        fail(f"{len(files) + 1} files: a native bundle holds at most {MAX_NATIVE_FILES}")

    entries: dict[str, bytes] = {
        "horizon-native.json": (json.dumps(meta, indent=2, sort_keys=True) + "\n").encode()
    }
    total = len(entries["horizon-native.json"])
    for rel, path in files:
        data = path.read_bytes()
        if len(data) > MAX_FILE:
            fail(f"{rel}: over 64 MiB")
        total += len(data)
        entries[f"mixins/{meta['id']}/{rel.as_posix()}"] = data
    if total > MAX_NATIVE_BYTES:
        fail(f"unpacked content is {total} bytes: over 64 MiB")

    required = f"mixins/{meta['id']}/horizon-mixin.toml"
    if required not in entries:
        fail("the bundle's horizon-mixin.toml was not packed")

    output = output or bundle / "target" / f"{meta['id']}-{meta['version']}.hnmod"
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_suffix(output.suffix + ".partial")
    with zipfile.ZipFile(partial, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(entries):
            info = zipfile.ZipInfo(name, (2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, entries[name])
    partial.replace(output)
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    print(
        json.dumps(
            {
                "file": str(output),
                "size": output.stat().st_size,
                "sha256": digest,
                "id": meta["id"],
                "version": meta["version"],
                **({"hooks": meta["hooks"]} if "hooks" in meta else {}),
            },
            indent=2,
        )
    )


def check(path: Path) -> None:
    if not path.is_file():
        fail(f"{path}: no such file")
    if path.stat().st_size > MAX_FILE:
        fail(f"{path}: over 64 MiB")
    try:
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
            entries = {name: archive.read(name) for name in names}
    except zipfile.BadZipFile as e:
        fail(f"{path}: not a zip: {e}")

    if "horizon-native.json" not in entries:
        fail("no horizon-native.json")
    try:
        meta = json.loads(entries["horizon-native.json"])
    except json.JSONDecodeError as e:
        fail(f"horizon-native.json: {e}")
    if not isinstance(meta, dict):
        fail("horizon-native.json: not an object")
    known = {"schema", "id", "version", "game", "horizon", "mixins", "hooks"}
    unknown = set(meta) - known
    if unknown:
        fail(f"horizon-native.json: unknown fields {sorted(unknown)}")
    if meta.get("schema") != 1:
        fail("horizon-native.json: schema is not 1")
    mod_id = str(meta.get("id", ""))
    if not MOD_ID.fullmatch(mod_id):
        fail("horizon-native.json: bad id")
    for name in ("horizon", "mixins"):
        if not PIN.fullmatch(str(meta.get(name, ""))):
            fail(f"horizon-native.json: bad {name}")
    if "hooks" in meta and (not isinstance(meta["hooks"], int) or meta["hooks"] < 1):
        fail("horizon-native.json: bad hooks")
    if not meta.get("version") or not meta.get("game"):
        fail("horizon-native.json: empty version or game")

    if len(names) > MAX_NATIVE_FILES:
        fail(f"{len(names)} entries: over {MAX_NATIVE_FILES}")
    if len(names) != len(set(names)):
        fail("duplicate zip entries")
    total = 0
    for name in names:
        if name.endswith("/"):
            fail(f"{name}: directory entries are not allowed")
        rel = PurePosixPath(name)
        if name == "horizon-native.json":
            continue
        if not name.startswith(f"mixins/{mod_id}/"):
            fail(f"{name}: outside mixins/{mod_id}/")
        inner = PurePosixPath(*rel.parts[2:])
        if not allowed(inner):
            fail(f"{name}: not allowed in a native bundle")
        total += len(entries[name])
    if total > MAX_NATIVE_BYTES:
        fail("unpacked content is over 64 MiB")

    manifest_name = f"mixins/{mod_id}/horizon-mixin.toml"
    if manifest_name not in entries:
        fail(f"no {manifest_name}")
    try:
        mixin = tomllib.loads(entries[manifest_name].decode())
    except (UnicodeDecodeError, tomllib.TOMLDecodeError) as e:
        fail(f"{manifest_name}: {e}")
    mod = mixin.get("mod", {})
    if mod.get("id") != mod_id:
        fail(f"{manifest_name}: [mod] id does not match horizon-native.json")
    if str(mod.get("version")) != str(meta.get("version")):
        fail(f"{manifest_name}: [mod] version does not match horizon-native.json")
    pinned = mixin.get("compatibility", {}).get("horizon_revision")
    if pinned is None:
        fail(f"{manifest_name}: no [compatibility] horizon_revision")
    if pinned != meta.get("horizon"):
        fail(f"{manifest_name}: horizon_revision does not match horizon-native.json")

    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    print(
        json.dumps(
            {
                "ok": True,
                "file": str(path),
                "size": path.stat().st_size,
                "sha256": digest,
                "id": mod_id,
                "version": meta.get("version"),
                **({"hooks": meta["hooks"]} if "hooks" in meta else {}),
            },
            indent=2,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", help="bundle folder (or .hnmod with --check)")
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--check", action="store_true", help="check an .hnmod instead of packing")
    parser.add_argument("--game", help="Minecraft version (default: compatibility.json or 26.3)")
    parser.add_argument("--horizon", help="assert the expected Horizon commit")
    parser.add_argument("--mixins", help="horizon-mixins commit when compatibility.json is absent")
    parser.add_argument("--hooks", type=int, help="required native hook points version")
    args = parser.parse_args()
    path = Path(args.bundle)
    if args.check:
        check(path)
    else:
        package(path, args.output, args)


if __name__ == "__main__":
    main()
