# FTC project instructions

- This is an FTC Android Studio Java project. Put team robot code in TeamCode.
- Inspect the existing OpModes, hardware map names, units and conventions before editing.
- Preserve the FTC SDK, Gradle wrapper, Android Gradle plugin and Java versions unless explicitly asked to upgrade them.
- Prefer small, reviewable changes. Do not edit generated files, local.properties or credentials.
- Keep control loops responsive; check opModeIsActive()/isStopRequested() as appropriate and avoid unbounded waits.
- Ensure motor and servo commands stay within the intended bounds and stop mechanisms on OpMode termination.
- Ask for missing hardware names, gear ratios, encoder specifications or control requirements instead of inventing them.
- For a build check, use the existing Windows wrapper: .\gradlew.bat :TeamCode:assembleDebug.
- Do not deploy to a Robot Controller, run an OpMode or actuate hardware without an explicit request.
- Report changed files and validation results, including any build or hardware checks that could not be performed.