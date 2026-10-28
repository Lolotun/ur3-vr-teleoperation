"""Validate the local patch without changing the installed Unity package."""
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('patch_meta_xr',Path(__file__).resolve().parents[1]/'scripts/patch-meta-xr.py')
patcher=importlib.util.module_from_spec(spec)
spec.loader.exec_module(patcher)

ORIGINAL='''namespace Example {
        private static async Task<bool> DownloadXRSimInstaller(string url, Action onSuccess, Action<string> onError)
        {
#if UNITY_EDITOR_WIN
            var downloadedInstallerPath = "installer.msi";
#elif UNITY_EDITOR_OSX
            var downloadedInstallerPath = "installer.dmg";
#endif
            await DownloadInstaller(downloadedInstallerPath, url, onError);
            onSuccess?.Invoke();
            return true;
        }

        internal static async Task<bool> DownloadInstaller(string path, string url, Action<string> onError)
        {
            return true;
        }
}
'''


class MetaPatchTests(unittest.TestCase):
    def test_original_is_guarded_and_idempotent(self):
        result=patcher.patch_installer(ORIGINAL)
        self.assertIn('#if UNITY_EDITOR_WIN || UNITY_EDITOR_OSX',result)
        self.assertIn('return false;\n#endif',result)
        self.assertEqual(patcher.patch_installer(result),result)
        tail=ORIGINAL[ORIGINAL.index('internal static async Task<bool> DownloadInstaller('):]
        self.assertTrue(result.endswith(tail))

    def test_older_workaround_is_replaced(self):
        older=ORIGINAL.replace('#endif','#else\n            var downloadedInstallerPath = string.Empty;\n#endif',1)
        self.assertEqual(patcher.patch_installer(older),patcher.patch_installer(ORIGINAL))

    def test_unknown_layout_fails_without_modifying_input(self):
        with self.assertRaises(ValueError): patcher.patch_installer('unrelated code')
        with self.assertRaises(ValueError): patcher.patch_installer(ORIGINAL.replace('#elif UNITY_EDITOR_OSX','#elif SOMETHING_ELSE'))


if __name__=='__main__': unittest.main()
