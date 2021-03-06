# Haptic Learning
A simple proof of concept for the Haptic Tutor project.

## Settings
- In `Audio MIDI Setup` it is recommended to create a new port in the `IAC Driver`.
	- Other method are possible and will vary based on Operating System. __This is not a promotion of Apple products.__

<img src="imgs/audio-midi settings.png" width=500>

- In `Logic Pro`, create a new `External MIDI` instrument and set the MIDI Destination accordingly.
	- Other Audio Editing Software can also accomplish this task. __This is not a promotion of Apple products.__

<img src="imgs/logic settings.png" width=500>

- Make sure the MIDI Destination matches in MAX.
	- Other Software can also accomplish this task. __This is not a promotion of Cycling '74 software.__

<img src="imgs/Max settings.png" width=500>

## Source files
- `/Tactons.maxpat` : Application (MaxMSP build) that parses and interprets MIDI messages from `/drum1.maxpat` and `ins-filter.maxpat`
- `/vibropixels/VP_Metro` includes device drivers and communication protocol written in C++

