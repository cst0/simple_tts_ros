#!/usr/bin/env python3

import sys
import os
import json

import rospy

from simple_tts.msg import Speak
from simple_tts.srv import SpeechRequest, SpeechRequestRequest, SpeechRequestResponse

DEFAULT_TTS_ENGINE = "gtts"


class SpeechEngine(object):
    def __init__(self, speech_engine=None):

        if speech_engine is None:
            rospy.loginfo("No engine specified: defaulting to " + DEFAULT_TTS_ENGINE)
            speech_engine = DEFAULT_TTS_ENGINE

        rospy.loginfo("Engine selected: " + speech_engine)

        config_path = os.path.abspath(
            os.path.dirname(os.path.abspath(__file__)) + "/../config/engines.json"
        )
        with open(config_path) as f:
            config = json.loads("".join(f.readlines()))
            if speech_engine not in config:
                rospy.logerr(
                    "Specified speech engine ("
                    + speech_engine
                    + ") not in engines.json: defaulting to "
                    + DEFAULT_TTS_ENGINE
                )
            speech_engine = DEFAULT_TTS_ENGINE

        # take the speech_engine string and associate it with a directory
        engine_path = os.path.abspath(
            os.path.dirname(os.path.abspath(__file__)) + "/engines/" + speech_engine
        )
        sys.path.append(engine_path)
        try:
            from engine import Engine  # type: ignore (we're doing some funky stuff with import paths here, see README)
        except:
            rospy.logfatal(
                "Unable to load speech engine wrapper for '"
                + speech_engine
                + "'. Tried looking here:\n\t"
                + engine_path
                + "\nDoes it exist?"
            )
            sys.exit()

        self.speech_engine = Engine()

        self.text_input_subscriber = rospy.Subscriber(
            "tts_in", Speak, self.text_input_callback
        )

        self.text_input_service = rospy.Service(
            "call_tts", SpeechRequest, self.text_input_server
        )

    def text_input_callback(self, msg: Speak):
        self.speech_engine.say(msg.text)

    def text_input_server(self, req: SpeechRequestRequest):
        self.speech_engine.say(req.text)
        return SpeechRequestResponse()

    def shutdown(self):
        self.text_input_subscriber.unregister()
        self.text_input_service.shutdown()
        self.speech_engine.shutdown()


def main():
    rospy.init_node("tts", anonymous=False)
    engine_param = rospy.get_param("engine", default=None)
    se = SpeechEngine(engine_param)
    rospy.on_shutdown(se.shutdown)

    rospy.loginfo("Ready to perform tts!")
    rospy.spin()
    rospy.loginfo("TTS node told to shut down. Goodbye!")


if __name__ == "__main__":
    main()
