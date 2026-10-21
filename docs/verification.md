# Verification record

Checked on 21 October 2026.

## Original files

The optimized sketch and existing HEX were collected from the desktop `SolarPanelControl_Optimized` project. The older `Task1` sketch and HEX were recovered from the Arduino folder named by the Proteus firmware path. The circuit came from the RMI intermediate solar-panel exercise.

All five repository copies match their original SHA-256 hashes. The source code and binaries were not edited. `source-inventory.json` contains each filename, byte count, role and hash.

The optimized version adds conditional integral accumulation, a direction-sign setting and floating-point magnitude handling. The older source remains available as the revision referenced by the saved Proteus firmware path. Finding a source and binary in the same folder does not establish that the binary was compiled from the current source.

## Circuit checks

The `.pdsprj` is a valid ZIP archive containing `ROOT.CDB`, `ROOT.DSN`, `PROJECT.XML`, `GRAPHS.DAT` and a power-rail data file. All archive entries passed CRC checks, and the project XML parsed.

Stored component/property strings identify an Uno/ATmega328P model, LDRs, L293D and a ±70° motor-angle range. The saved Program File points to the older `Task1.ino.hex` at an absolute local path. It must be updated after cloning. Full pin-to-net connectivity and component-library availability have not been verified in Proteus.

## Historical HEX checks

Both files use valid Intel HEX records with matching lengths/checksums and a terminating EOF record. Data addresses do not overlap and remain below the Uno's 32768-byte flash boundary.

| Export | Data bytes | Highest populated address |
| --- | --- | --- |
| Optimized | 5090 | 5089 |
| Earlier Task1 | 5126 | 5125 |

These are file-format checks. They do not demonstrate program behavior, source/binary equivalence or RAM usage.

## Fresh build attempt

Attempted to compile the unchanged optimized sketch for `arduino:avr:uno` using the installed Arduino CLI 1.5.1, Arduino AVR Boards 1.8.8 and AVR GCC package `7.3.0-atmel3.6.1-arduino7`. Build/cache outputs were directed to the workspace.

The build failed before compilation when Windows tried to launch `avr-g++.exe`:

```text
{Illegal System DLL Relocation}
The system DLL ... was relocated in memory. The application will not run properly.
```

No new successful build or binary was produced. Existing exported HEX files are labeled historical rather than represented as newly built artifacts.

## Runtime scope

No Proteus simulation or hardware test was run during preparation. Scheduled task intervals, pin assignments, PID constants and LED thresholds in the README are derived from source inspection. They are not measured results.

Next meaningful verification is a fresh Uno build in a functioning AVR toolchain, followed by circuit wiring review and recorded closed-loop tests with changing target angle, travel limits, saturation, sensor noise and LED transitions.
