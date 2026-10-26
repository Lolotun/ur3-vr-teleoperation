#!/usr/bin/env python3
"""Restore the Meta XR 85 Linux compilation fix present in the workshop cache."""

import json
from pathlib import Path
import re


def main():
    project = Path(__file__).resolve().parents[1] / "unity"
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
        start = source.index("private static async Task<bool> DownloadXRSimInstaller(")
        end = source.index("#endif", start)
        conditional = source[start:end]
        if re.search(r"#else\s+var downloadedInstallerPath = string.Empty;", conditional):
            print(f"Already patched: {installer.relative_to(project)}")
            continue
        if "#else" in conditional or "#elif UNITY_EDITOR_OSX" not in conditional:
            raise SystemExit("Unexpected Installer.cs layout; review before patching.")
        installer.write_text(source[:end] + "#else\n    var downloadedInstallerPath = string.Empty;\n" + source[end:])
        print(f"Patched: {installer.relative_to(project)}")
    if not found:
        raise SystemExit("Meta XR Core 85.0.0 not found. Open unity/ and resolve packages first.")


if __name__ == "__main__":
    main()
