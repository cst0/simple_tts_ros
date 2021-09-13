import sys
import os

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
        temp = tempfile.NamedTemporaryFile("w+")
        tts = gTTS(text, "en")
        tts.save(temp.name)
        os.system("mpg321 " + self.temp.name) #TODO-- remove cli dependency
        self.speaking = False

    def is_speaking(self) -> bool:
        return self.speaking

    def shutdown(self):
        pass
