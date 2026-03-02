#!/usr/bin/env python3
"""
Push-to-talk voice input for MotionCorps using ElevenLabs Speech-to-Text.

Press ENTER to start recording, silence stops recording automatically.
Transcribed text is printed to stdout (captured by the shell) and also
echoed visibly to stderr so the user sees it immediately in the terminal.

Usage:
    python3 voice_input.py
"""

import io
import os
import sys
import time
import wave

import numpy as np
import requests
import sounddevice as sd

SAMPLE_RATE = 16000
SILENCE_THRESHOLD = 0.01   # RMS fraction (0–1) below this = silence
SILENCE_SECONDS = 1.8      # stop after this many seconds of silence
MAX_SECONDS = 15            # hard cap per utterance

ELEVENLABS_STT_URL = "https://api.elevenlabs.io/v1/speech-to-text"
ELEVENLABS_API_KEY = os.environ.get("ELEVENLABS_API_KEY", "")

# ANSI colours (written to stderr so they reach the terminal)
CYAN  = "\033[0;36m"
BOLD  = "\033[1m"
DIM   = "\033[2m"
GREEN = "\033[0;32m"
NC    = "\033[0m"


def err(msg):
    sys.stderr.write(msg + "\n")
    sys.stderr.flush()


def rms(chunk):
    return float(np.sqrt(np.mean(chunk.astype(np.float32) ** 2))) / 32768.0


def record_until_silence():
    """Record from microphone until silence, return raw int16 PCM array."""
    chunks = []
    silence_started = None
    started = time.monotonic()

    def callback(indata, frames, time_info, status):
        chunks.append(indata[:, 0].copy())

    with sd.InputStream(samplerate=SAMPLE_RATE, channels=1,
                        dtype="int16", blocksize=2048,
                        callback=callback):
        while True:
            time.sleep(0.05)
            elapsed = time.monotonic() - started
            if elapsed > MAX_SECONDS:
                break
            if not chunks:
                continue
            level = rms(chunks[-1])
            if level < SILENCE_THRESHOLD:
                if silence_started is None:
                    silence_started = time.monotonic()
                elif time.monotonic() - silence_started >= SILENCE_SECONDS:
                    break
            else:
                silence_started = None

    if not chunks:
        return None
    return np.concatenate(chunks)


def to_wav_bytes(pcm, sample_rate):
    """Wrap raw int16 PCM in a WAV container and return bytes."""
    buf = io.BytesIO()
    with wave.open(buf, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)       # int16 = 2 bytes
        wf.setframerate(sample_rate)
        wf.writeframes(pcm.tobytes())
    return buf.getvalue()


def transcribe_elevenlabs(wav_bytes):
    """Send WAV audio to ElevenLabs STT and return transcribed text."""
    resp = requests.post(
        ELEVENLABS_STT_URL,
        headers={"xi-api-key": ELEVENLABS_API_KEY},
        files={
            "file": ("audio.wav", wav_bytes, "audio/wav"),
        },
        data={
            "model_id": "scribe_v1",
        },
        timeout=30,
    )
    if not resp.ok:
        err(f"{DIM}ElevenLabs response: {resp.text}{NC}")
        resp.raise_for_status()
    return resp.json().get("text", "").strip()


def main():
    if not ELEVENLABS_API_KEY:
        err("ERROR: ELEVENLABS_API_KEY not found. Source .env first.")
        sys.exit(1)

    err(f"{CYAN}ElevenLabs STT ready.{NC} Press {BOLD}ENTER{NC} to speak.\n")

    while True:
        try:
            input()         # block until Enter is pressed
        except EOFError:
            break

        err(f"{BOLD}🎙  Recording...{NC} (speak now, silence stops)")
        pcm = record_until_silence()

        if pcm is None or len(pcm) < int(SAMPLE_RATE * 0.3):
            err("(no audio captured — try again)")
            continue

        duration = len(pcm) / SAMPLE_RATE
        err(f"{DIM}Captured {duration:.1f}s — sending to ElevenLabs...{NC}")

        try:
            wav_bytes = to_wav_bytes(pcm, SAMPLE_RATE)
            text = transcribe_elevenlabs(wav_bytes)
        except requests.HTTPError as e:
            err(f"ElevenLabs STT error: {e}")
            continue
        except Exception as e:
            err(f"Transcription error: {e}")
            continue

        if not text:
            err("(nothing recognised — try again)")
            continue

        # Show the transcription prominently in the terminal
        err(f"\n{GREEN}{BOLD}You said:{NC} {text}\n")

        # Send to stdout so the shell script captures it as the command
        print(text, flush=True)


if __name__ == "__main__":
    main()
