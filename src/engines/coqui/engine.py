#!/usr/bin/env python3
import sys
import os
import subprocess

repo_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + "/..")
sys.path.append(repo_path)
from ttsengine import TTSEngine

from gtts import gTTS
import tempfile


class Engine(TTSEngine):
    def __init__(self):
        self.speaking = False

    def say(self, text: str):
        self.speaking = True
        with tempfile.NamedTemporaryFile("w+", suffix=".wav") as temp:
            # coqui's models can get funky if utterances aren't terminated with a single full-stop
            if not text.endswith('.'):
                text = text + '.'

            # coqui is made available via this cli. Don't love doing this but it works.
            subprocess.check_output(["tts", "--text", text, "--out_path", temp.name])
            subprocess.check_output(["aplay", temp.name]) # TODO-- remove OS dependency
        self.speaking = False

    def is_speaking(self) -> bool:
        return self.speaking

    def shutdown(self):
        pass
