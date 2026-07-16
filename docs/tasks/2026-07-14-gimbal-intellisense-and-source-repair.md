# Gimbal Intellisense And Source Repair

## Requirement Summary

Resolve the missing-header and cascading declaration diagnostics in the MSPM0
Gimbal module shown by VS Code.

## Confirmed Approach

- Add explicit MSPM0 firmware include directories to the workspace C/C++ configuration.
- Restore declarations and statements in `Gimbal.c` that were merged onto comment lines.
- Keep UART4 parsing and yaw/pitch control behavior unchanged.

## Files To Change

- `.vscode/c_cpp_properties.json`
- `MSPM0G3519/project/user/src/Gimbal.c`

## Verification Checklist

- Confirm all required local headers resolve with the configured include paths.
- Run a syntax-only compilation of `Gimbal.c` using those paths.

## Decision

The source file has five merged lines where `//` comments hide valid C code;
these are repaired as a source defect rather than suppressed in the editor.
