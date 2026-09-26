# bpm

A single-file C command-line BPM estimator with an interactive waveform TUI. It builds on Windows, macOS, and Linux. The BPM estimator reports when the tempo varies or the fit is unstable, so the result may be inaccurate.

## Build

The source requires a C11 compiler and the platform's standard system libraries. Run these commands from the repository directory:

```sh
# Windows (GCC / MinGW-w64)
gcc -std=c11 -O2 -o bpm.exe bpm.c -lm -lwinmm

# macOS (Clang)
cc -std=c11 -O2 -o bpm bpm.c -lm

# Linux (GCC or Clang)
cc -std=c11 -O2 -o bpm bpm.c -lm
```

No third-party library is needed to build. Supported PCM WAV files can be analyzed directly. For MP3 and other unsupported formats, install `ffmpeg` and put it on `PATH`; the program will use it to decode to a temporary WAV file.

TUI playback uses the Windows audio API on Windows, the built-in `afplay` command on macOS, and `ffplay` or `aplay` on Linux. Playback on macOS/Linux requires the corresponding player to be available on `PATH`. The macOS/Linux playhead uses elapsed time and may differ slightly from the player's actual output during startup.

## Command-line usage

```sh
bpm [--offset|-o|o] <audio-file>
bpm --tui [audio-file]
bpm --help
```

On Windows, run `bpm.exe` instead of `bpm` (for example, `.\bpm.exe --tui "song.mp3"`). On macOS/Linux, run `./bpm` when the executable is in the current directory.

- `--offset`, `-offset`, `--o`, `-o`, or `o`: also print the leading-silence offset in milliseconds.
- `--tui`: open the waveform interface. If the audio path is omitted, the program prompts for it before opening the TUI.
- `--help`, `--h`, or `-h`: show built-in help.

The estimator accepts integer PCM WAV (8, 16, 24, or 32 bit) and 32-bit float WAV directly at 32 kHz, 44.1 kHz, or 48 kHz. Other formats and sample rates are passed through `ffmpeg` when available.

## TUI controls

The vertical `#` line stays at the current time; the waveform scrolls beneath it. An interactive terminal with mouse reporting is required.

| Input | Action |
| --- | --- |
| Space | Play or pause from the displayed time. |
| Mouse wheel | Move through the audio. Scrolling stops playback. |
| Left click, then left click | Set the start and end of a measurement range. Multiple ranges are supported. |
| Right click inside a range | Delete that range. Remaining ranges are renumbered. |
| Type `get` and press Enter | Exit the TUI and print BPM and offset for each range. |
| Type `offset` and press Enter | Show the audio's leading-silence offset in the TUI. |
| Home / End | Jump to the beginning / end. |
| Ctrl+C | Exit the TUI. |

After `get`, each result is printed as:

```text
1.BPM:120.0,offset:15.0 ms;
2.BPM:128.0,offset:0.0 ms;
```

In these results, offset is measured from the start of each selected range. The command-line `--offset` value is measured from the start of the complete audio file.
