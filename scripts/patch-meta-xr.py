#!/usr/bin/env python3
"""Apply the Meta XR Core 85 Linux installer guard without vendoring the SDK."""

import argparse
import json
from pathlib import Path
import re


def patch_installer(source):
    signature = "private static async Task<bool> DownloadXRSimInstaller("
    following = "internal static async Task<bool> DownloadInstaller("
    try:
        start = source.index(signature)
        body_start = source.index("{", start) + 1
        next_method = source.index(following, body_start)
        body_end = source.rindex("}", body_start, next_method)
    except ValueError as exc:
        raise ValueError("Unexpected Installer.cs layout; review before patching.") from exc
    body = source[body_start:body_end]
    if body.lstrip().startswith("#if UNITY_EDITOR_WIN || UNITY_EDITOR_OSX"):
        if '#else\n            onError?.Invoke("Meta XR Simulator installer is only available on Windows and macOS.");' not in body:
            raise ValueError("Unexpected existing platform guard; review before patching.")
        return source
    # Replace the older empty-path workaround with a guard around the entire method.
    body = re.sub(r"#else\s+var downloadedInstallerPath = string.Empty;\s*(?=#endif)", "", body)
    if not body.lstrip().startswith("#if UNITY_EDITOR_WIN") or "#elif UNITY_EDITOR_OSX" not in body:
        raise ValueError("Unexpected platform conditions; review before patching.")
    guarded = ("\n#if UNITY_EDITOR_WIN || UNITY_EDITOR_OSX" + body.rstrip() +
               '\n#else\n            onError?.Invoke("Meta XR Simulator installer is only available on Windows and macOS.");'
               '\n            return false;\n#endif\n        ')
    return source[:body_start] + guarded + source[body_end:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path,
                        default=Path(__file__).resolve().parents[1] / "unity")
    project = parser.parse_args().project
    candidates = list((project / "Library/PackageCache").glob("com.meta.xr.sdk.core*"))
    candidates.append(project / "Packages/com.meta.xr.sdk.core")
    found = False
    for package in candidates:
        metadata = package / "package.json"
        if not metadata.is_file():
            continue
        if json.loads(metadata.read_text())["version"] != "85.0.0":
            continue
        found = True
        installer = package / "Editor/MetaXRSimulator/Installer.cs"
        source = installer.read_text()
        try:
            patched = patch_installer(source)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc
        if patched == source:
            print(f"Already patched: {installer.relative_to(project)}")
            continue
        installer.write_text(patched)
        print(f"Patched: {installer.relative_to(project)}")
    if not found:
        raise SystemExit("Meta XR Core 85.0.0 not found. Open unity/ and resolve packages first.")


if __name__ == "__main__":
    main()
