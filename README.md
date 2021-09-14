# ROS Simple TTS

This package provides easy-to-use wrappers for a handful of text-to-speech
engines under a unified interface. TTS can be performed via a ROS service,
publishing to a ROS topic, or by importing the python module and making calls
directly.

The following TTS engines are supported:

- gtts
- coqui
- marytts
- pyttsx3

# Dependencies
For the sake of reducing dependencies, all supported engines are not included
as dependencies (since you'll likely be using only one anyway).  The engines
can be acquired via pip:

```
pip install gtts
pip install marytts
pip install pyttsx3
```

# Installation

## From Source

clone and then catkin_make.

## As a binary

Coming soon, probably.

# Usage

## Quickstart

A launch file is provided that should work out-of-the-box:

```
roslaunch simple_tts simple_tts.launch
```

The `tts_in` topic should now be available. Publishing to this topic will lead
to the node attempting to speak the text.  As a simple demonstration, try
running:

```
rostopic pub /tts_in std_msgs/String "data: 'hello world'"
```

The text should now be spoken.

## Configuration
The parameter "engine" allows for different engines to be used.
