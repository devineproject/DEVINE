Audio
#####

Description
===========

We use `SNIPS` as our voice assistant to interact with the robot with the voice.

ROS Installation
================

As `SNIPS` does not officially support Ubuntu Xenial, its intallation comes with a few caveat.

1. Run ``$ sudo npm install -g snips-sam`` to install SAM
2. Go to /usr/lib/node_modules/snips-sam/lib/session/ssh.js (or usr/local/lib/node_modules/snips-sam/lib/session/ssh.js) and change line 426 to [...] ``echo "deb https://debian.snips.ai/stretch stable main"`` [...]
3. Install an upstream version of libc ``$ sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test && sudo apt-get update && sudo apt-get upgrade -y libstdc++6``
4. Connect with ``$ sam connect localhost``
5. ``$ sam init``
6. If you get an error at this stage, add this line ``your_username ALL=(ALL) NOPASSWD: ALL`` at the end of your sudoers file with the command : ``$ sudo visudo``, then try again from step 4.
7. Test the speaker with ``$ sam test speaker``
8. Test the microphone with ``$ sam test microphone``
9. If tests are not conclusive or quality is poor, try selecting a different speaker and microphone with : ``$ sam setup audio``
10. Install our assistant ``$ wget https://github.com/projetdevine/static/releases/download/v0.0.1/assistant.zip && sudo unzip -o assistant.zip -d /usr/share/snips``

Once the `SNIPS` team adds support for Ubuntu Xenial step 2 and 3 will not be necessary. Note that our assistant was tested for version 0.58.3 of the snips-platform-voice package.

Usage
================

.. code-block:: bash

  $ roscore #start ROS master
  $ rosrun devine_dialog snips.py __ns:=devine #run snips node
  $ sam watch
  $ rostopic echo /devine/tts/answer #listen to the answers



To send custom data to the topic used by snips, do :

``$ rosrun rqt_gui rqt_gui``

- Select topic : /devine/tts/query
- Select type : devine_dialog/TtsQuery
- Select a frequency
- Fill out the 'text' (ex: "Is the object blue ?"), 'uid' (ex: 1) and 'answer_type' (ex: 1) fields.

Or, run this command :
``$ rostopic pub /devine/tts/query devine_dialog/TtsQuery '{text: "Is the object blue?", uid: 1, answer_type: 1}'``


.. _SNIPS: https://snips.ai/
