# CLAUDE.md

This file gives Claude Code project-specific navigation rules for this STM32/CubeMX firmware repository.

## Project scope

The effective project root is `D:\2026_r2_v1`, not `D:\2026_r2_v1\MDK-ARM`.

When answering code questions, prefer source/configuration files from the whole project root, especially:

- `Core/**/*.c`
- `Core/**/*.h`
- `Drivers/**/*.c`
- `Drivers/**/*.h`
- `Middlewares/**/*.c`
- `Middlewares/**/*.h`
- `user_lib/**/*.c`
- `user_lib/**/*.h`
- `cmsis_dsp/**/*.c`
- `cmsis_dsp/**/*.h`
- `MDK-ARM/**/*.s`
- `MDK-ARM/**/*.sct`
- `*.ioc`
- `.mxproject`
- `CMakeLists.txt`
- `Makefile`

## Search policy

Do not broad-glob the whole repository unless the user explicitly asks for a full project inventory.

For normal code questions:

1. Start with targeted `Grep` searches over `*.c`, `*.h`, `*.ioc`, `*.s`, `*.sct`, and project config files.
2. Prefer searching likely source directories first: `Core/`, `user_lib/`, `Drivers/`, `Middlewares/`, `cmsis_dsp/`, and then `MDK-ARM/` only when toolchain/project configuration is relevant.
3. Avoid reading generated build outputs, object files, dependency files, listing files, logs, IDE user-state files, and backup files.
4. If a broad search is unavoidable, explain why and use restrictive glob filters.

## Files and folders to ignore by default

Ignore these unless the user explicitly asks about build artifacts, Keil project internals, or generated outputs:

- `Debug/`
- `Release/`
- `build/`
- `cmake-build-*/`
- `.git/`
- `.vscode/`
- `.idea/`
- `MDK-ARM/2026_r2_v1/`
- `MDK-ARM/DebugConfig/`
- `MDK-ARM/RTE/`
- `*.o`
- `*.obj`
- `*.elf`
- `*.axf`
- `*.bin`
- `*.hex`
- `*.map`
- `*.lst`
- `*.list`
- `*.crf`
- `*.d`
- `*.dep`
- `*.su`
- `*.bak`
- `*.log`
- `*.uvguix.*`
- `*.scvd`
- `*.dbgconf`

## Communication preference

When discussing code locations, reference files using paths relative to `D:\2026_r2_v1`.

## STM32 project conventions

This is an STM32H723VGT6 robot firmware project developed with CubeMX + Keil + VSCode.

When locating code:
- Treat `D:\2026_r2_v1` as the project root.
- Prefer `user_lib/` for robot behavior, chassis, arm, lift, planner, and custom modules.
- Prefer `Core/Src/` and `Core/Inc/` for CubeMX-generated init code and FreeRTOS task creation.
- Prefer `.ioc` for peripheral configuration.
- Check `MDK-ARM/*.uvprojx` only when Keil project inclusion/build configuration is relevant.
- Ignore build outputs unless explicitly requested.
