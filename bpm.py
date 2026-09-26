#!/usr/bin/env python3
"""Estimate a track's tempo, or report the silence before its first sound.

Usage:
    python bpm.py [--offset | -offset] <audio-path>

WAV files are read directly. Other formats are decoded through ffmpeg when it
is installed. The implementation uses only Python's standard library.
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
import wave
from array import array
from pathlib import Path


class AudioError(Exception):
    """An audio file could not be loaded or analyzed."""


def _read_wav(path: Path) -> tuple[int, int, int, bytes]:
    try:
        with wave.open(str(path), "rb") as audio:
            channels = audio.getnchannels()
            sample_width = audio.getsampwidth()
            sample_rate = audio.getframerate()
            frames = audio.readframes(audio.getnframes())
    except (wave.Error, EOFError) as exc:
        raise AudioError(f"无法读取 WAV 音频：{exc}") from exc
    if sample_width not in (1, 2, 3, 4):
        raise AudioError(f"不支持 {sample_width * 8} 位 WAV 音频")
    return sample_rate, channels, sample_width, frames


def _load_audio(path: Path) -> tuple[int, int, int, bytes]:
    if path.suffix.lower() in (".wav", ".wave"):
        return _read_wav(path)

    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise AudioError("该音频格式需要 ffmpeg；也可以先转换为 WAV")
    try:
        result = subprocess.run(
            [ffmpeg, "-v", "error", "-i", str(path), "-f", "wav", "-acodec", "pcm_s16le", "-"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except (subprocess.CalledProcessError, OSError) as exc:
        detail = exc.stderr.decode("utf-8", errors="replace").strip() if getattr(exc, "stderr", None) else str(exc)
        raise AudioError(f"ffmpeg 无法解码音频：{detail}") from exc
    import io

    try:
        with wave.open(io.BytesIO(result.stdout), "rb") as audio:
            return audio.getframerate(), audio.getnchannels(), audio.getsampwidth(), audio.readframes(audio.getnframes())
    except (wave.Error, EOFError) as exc:
        raise AudioError(f"ffmpeg 输出的 WAV 无效：{exc}") from exc


def _mono_samples(sample_width: int, channels: int, frames: bytes) -> array:
    if sample_width == 1:
        raw = array("B", frames)
        values = array("h", (sample - 128 for sample in raw))
    elif sample_width == 2:
        values = array("h")
        values.frombytes(frames)
        if sys.byteorder != "little":
            values.byteswap()
    elif sample_width == 3:
        values = array("i")
        for index in range(0, len(frames), 3):
            sample = int.from_bytes(frames[index:index + 3], "little", signed=True)
            values.append(sample >> 8)
    else:
        values = array("i")
        values.frombytes(frames)
        if sys.byteorder != "little":
            values.byteswap()
        values = array("i", (sample >> 16 for sample in values))
    if channels <= 1:
        return values
    return array("i", (sum(values[index:index + channels]) // channels for index in range(0, len(values) - channels + 1, channels)))


def _window_energy(samples: array, start: int, end: int) -> float:
    if end <= start:
        return 0.0
    return sum(sample * sample for sample in samples[start:end]) / (end - start)


def find_offset_ms(sample_rate: int, samples: array) -> float:
    """Return the start of sustained sound, ignoring quiet leading noise."""
    frame_size = max(1, sample_rate // 100)  # 10 ms windows
    peak = max((abs(value) for value in samples), default=0)
    if peak == 0:
        raise AudioError("音频全为静音，无法测量延迟")
    threshold = max(peak * 0.002, 80)
    for start in range(0, len(samples), frame_size):
        if _window_energy(samples, start, min(start + frame_size, len(samples))) >= threshold * threshold:
            next_start = start + frame_size
            if next_start >= len(samples) or _window_energy(samples, next_start, min(next_start + frame_size, len(samples))) >= threshold * threshold:
                return start * 1000.0 / sample_rate
    raise AudioError("没有检测到有效音频信号")


def estimate_bpm(sample_rate: int, samples: array) -> float:
    """Estimate tempo by autocorrelating a coarse RMS onset envelope."""
    if len(samples) < sample_rate * 2:
        raise AudioError("音频太短，至少需要约 2 秒才能估算 BPM")
    hop = max(1, sample_rate // 100)  # 100 values per second
    envelope = []
    for start in range(0, len(samples), hop):
        end = min(start + hop, len(samples))
        envelope.append(math.sqrt(_window_energy(samples, start, end)))

    # Positive energy changes emphasize attacks while suppressing sustained notes.
    onset = [0.0]
    onset.extend(max(0.0, envelope[i] - envelope[i - 1]) for i in range(1, len(envelope)))
    mean = sum(onset) / len(onset)
    onset = [value - mean for value in onset]

    min_bpm, max_bpm = 50.0, 250.0
    min_lag = max(1, int(60.0 * 100 / max_bpm))
    max_lag = min(len(onset) // 2, int(60.0 * 100 / min_bpm))
    if max_lag <= min_lag:
        raise AudioError("音频太短，无法估算 BPM")
    correlations = []
    for lag in range(min_lag, max_lag + 1):
        score = sum(onset[index] * onset[index - lag] for index in range(lag, len(onset)))
        correlations.append((score, lag))
    _, best_lag = max(correlations)
    bpm = 60.0 * 100 / best_lag
    # A strong half/double-time ambiguity is common; prefer the conventional range.
    if bpm > 180 and bpm / 2 >= min_bpm:
        bpm /= 2
    elif bpm < 70 and bpm * 2 <= max_bpm:
        bpm *= 2
    return bpm


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="bpm",
        description="测量音频 BPM；使用 --offset 或 -offset 测量开头静音延迟。",
        add_help=False,
    )
    parser.add_argument("-h", "--h", "--help", action="help", help="显示此帮助信息并退出")
    parser.add_argument("--offset", "-offset", action="store_true", help="输出音频开头静音延迟（毫秒）")
    parser.add_argument("path", type=Path, help="音频文件路径")
    args = parser.parse_args(argv)

    if not args.path.is_file():
        print(f"错误：找不到音频文件：{args.path}", file=sys.stderr)
        return 2
    try:
        sample_rate, channels, sample_width, frames = _load_audio(args.path)
        samples = _mono_samples(sample_width, channels, frames)
        if not samples or sample_rate <= 0:
            raise AudioError("音频文件没有可用的采样数据")
        if args.offset:
            print(f"{find_offset_ms(sample_rate, samples):.1f} ms")
        else:
            print(f"{estimate_bpm(sample_rate, samples):.1f} BPM")
    except AudioError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
