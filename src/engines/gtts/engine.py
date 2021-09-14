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
        with tempfile.NamedTemporaryFile("w+", suffix=".mp3") as temp:
            tts = gTTS(text, "com") # use translate.google.com instead of translate.google.en
            tts.save(temp.name)
            subprocess.check_output(["mpg123", temp.name]) # TODO-- remove OS dependency

        self.speaking = False

    def is_speaking(self) -> bool:
        return self.speaking

    def shutdown(self):
        pass
