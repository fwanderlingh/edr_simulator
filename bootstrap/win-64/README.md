# Private Windows bootstrap libraries

These unmodified x64 Visual C++ runtime DLLs let the pinned Windows micromamba
executable start without a system-wide VC Redistributable installation.
`start-simulator.ps1` verifies their SHA-256 hashes and copies them next to the
executable in `.tools/Library/bin` before invoking it. The simulator's Python
environment subsequently supplies its own VC runtime through the package lock.
Windows 10/11 supplies the Universal CRT and other Windows system DLLs.

Source: conda-forge package `vc14_runtime-14.51.36247-habf1de7_41.conda`, also
pinned in `../../environment-win-64.lock`:

https://conda.anaconda.org/conda-forge/win-64/vc14_runtime-14.51.36247-habf1de7_41.conda

Package SHA-256: `4e4cb599cdc41bf2109d1464c127b5bcbddf548ce3e322e612afb691338b48f8`

The DLLs were extracted from the package's `pkg-*.tar.zst`, and the included
license files were copied from `info/licenses` in its `info-*.tar.zst`.
`LICENSE.DOCX` contains the original Microsoft license; `LICENSE.RTF` points to
it, and `LICENSE.TXT` is the package's convenience text conversion.

| File | SHA-256 |
| --- | --- |
| `msvcp140.dll` | `7c26614e1d733892c2deac7e245ce115504b1d80592dd0a01b08e3e5a55f89ca` |
| `vcruntime140.dll` | `d1f4225df2cd877dbf130d5668a021dce3f94118455ff5ec952061c30afc9ce7` |
| `vcruntime140_1.dll` | `a7146c08f89fe5b04541ab507cdb59ff7b44534d4ba3c668a426c6450a03434e` |

When updating the runtime, obtain the package from conda-forge, verify its
checksum, preserve its license files, and update both these hashes and the
launcher. Test on Windows before distributing. These local libraries are
serviced through simulator updates, rather than Windows Update.
