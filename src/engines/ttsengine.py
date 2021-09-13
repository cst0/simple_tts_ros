import abc

class TTSEngine(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractclassmethod
    def say(self, text:str):
        raise NotImplementedError

    @abc.abstractclassmethod
    def is_speaking(self) -> bool:
        raise NotImplementedError

    @abc.abstractclassmethod
    def shutdown(self):
        raise NotImplementedError
