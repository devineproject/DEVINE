#!/bin/bash

mosquitto -d
snips-nlu &
snips-dialogue &
snips-tts &
snips-audio-server &
snips-asr
