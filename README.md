# ROS Simple TTS

This package provides easy-to-use wrappers for a handful of text-to-speech
engines under a unified interface. TTS can be performed via a ROS service,
publishing to a ROS topic, or by importing the python module and making calls
directly.

The following TTS engines are supported:

- gtts
- coqui
- ~~marytts~~ (in progress)
- ~~pyttsx3~~ (in progress)

# Dependencies
For the sake of reducing dependencies, all supported engines are not included
as dependencies (since you'll likely be using only one anyway).  The engines
can be acquired via pip:

```
pip install gtts
pip install tts
pip install marytts
pip install pyttsx3
```

Typical usage of these packages involves generating a .wav or .mp3 that can be
read from the command line. To perform this, two operating system dependencies
(`aplay` and `mpg321`) are also needed. Ubuntu systems already have `aplay` by
default. `mpg321` can be acquired via your package manager. Future plans
involve removing these operating-system dependencies.

If using `pyttsx3`, you will likely need to install its dependency of `libespeak`
(if on Ubuntu, this can be done via `sudo apt install libespeak-dev`).

# Installation

## From Source

This is a standard ros pacakge. Once the above dependencies have been resolved, clone into your catkin workspace and then run catkin_make.

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
