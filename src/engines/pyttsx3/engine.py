import sys
import os

repo_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + "/..")
sys.path.append(repo_path)
from ttsengine import TTSEngine

import pyttsx3

class Engine(TTSEngine):
    def __init__(self):
        self.eng = pyttsx3.init()
        self.eng.setProperty('rate', 125)

    def say(self, text: str):
        self.eng.say(text)
        self.runAndWait()
