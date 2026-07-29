---
name: wsl-powershell-shell
description: Use when OpenCode runs Linux shell commands, .sh scripts, Bash pipelines, or WSL tools from Windows PowerShell in this project.
---

# PowerShell to WSL shell execution

OpenCode shell commands for this project are launched by Windows PowerShell.
Route Linux commands explicitly through the intended WSL distribution instead
of invoking `bash`, `sh`, or a `.sh` file directly from PowerShell.

## Default form

Use `wsl.exe`, name the distribution, terminate WSL option parsing with `--`,
and pass the complete Linux command as one PowerShell single-quoted argument:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash -lc './tools/example.sh --flag value'
```

PowerShell single quotes keep Bash expressions such as `$HOME`, `$(command)`,
globs, pipes, and redirects intact for evaluation inside WSL:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash -lc 'printf "%s\n" "$HOME"; uname -s'
```

The OpenCode shell tool's WSL UNC `workdir` currently maps to the corresponding
Linux working directory. Set `workdir` to the project root. If a caller cannot
preserve that working directory, make it explicit in the Linux command:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash -lc 'cd /home/shm-white/ed && ./tools/example.sh'
```

## Separate argument form

In this environment, arguments appended after the command string were not
preserved by the plain `-- bash -lc` form. Put fixed arguments inside the
single-quoted command string. When arguments must remain separate, use
`--exec` and forward Bash's positional arguments explicitly:

```powershell
wsl.exe -d Ubuntu-24.04 --exec /bin/bash -lc 'exec /bin/bash "$@"' bash ./tools/example.sh --flag value
```

Do not interpolate untrusted input into the `bash -lc` command string. Use the
separate argument form for dynamic values and validate the target distribution
with `wsl.exe --list --quiet` when the machine or environment changes.

## Verified locally

- Distribution: `Ubuntu-24.04`
- Bash: `5.2.21(1)-release`
- Linux home: `/home/shm-white`
- Project mapping: `\\wsl.localhost\Ubuntu-24.04\home\shm-white\ed` to
  `/home/shm-white/ed`
- Actual script probe: `./tools/run_humble.sh --help` exited successfully using
  both the default form and the separate argument form.
