import sys
import os

repo_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + "/..")
sys.path.append(repo_path)
from ttsengine import TTSEngine

from gtts import gTTS
import tempfile


class Engine(TTSEngine):
    def __init__(self):
        pass

    def say(self, text: str):
        temp = tempfile.NamedTemporaryFile("w+")
        tts = gTTS(text, "en")
        tts.save(temp.name)
        os.system("mpg321 " + self.temp.name) #TODO-- remove cli dependency
