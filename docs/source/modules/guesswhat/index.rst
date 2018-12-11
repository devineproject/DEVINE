GuessWhat
#########

Description
===========

This project makes use of the open source code provided alongside the original `GuessWhat?!`_ research. On our side, we add the strict minimum to have it act as a ROS node.

.. _guesswhat?!: https://github.com/GuessWhatGame/guesswhat/

Installation
============

Since guesswhat is not yet a proper python module, it has to be added to your python path::

  $ git clone --recursive https://github.com/GuessWhatGame/guesswhat.git /tmp/somewhere
  $ export PYTHONPATH=/tmp/somewhere/src:$PYTHONPATH

Also install python dependencies::

  $ pip3 install -r requirements.txt

Build this ROS package using::

  $ catkin_make -C ~/catkin_ws

Usage
=====

Roslaunch::

  $ roslaunch devine devine.launch launch_all:=false guesswhat:=true

Monitor questions::

  $ rostopic echo /devine/tts/query
    text: "is it a person ?"
    uid: 1234
    answer_type: 1
    ---

Send some test inputs::

  $ cd example
  $ python3 example.py

Reply::

  $ rostopic pub /devine/tts/answer devine_dialog/TtsAnswer '{original_query: {text: "is it a person ?", uid: 1234, answer_type: 1}, probability: 1.0, text: "yes"}'
