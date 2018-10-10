Audio
#####

Description
===========

We use `SNIPS` as our voice assistant to interact with the robot with the voice.

ROS Installation
================

Installation with SAM

1. Run ``sudo npm install -g snips-sam`` to install SAM
2. Go to /usr/lib/node_modules/snips-sam/lib/session/ssh.js (or usr/local/lib/node_modules/snips-sam/lib/session/ssh.js) and change the line 426 to [...] ``echo “deb https://debian.snips.ai/stretch stable main”`` [...]
3. Connect with ``sam connect localhost``
4. ``sam init``
5. If you get an error at this stage, add this line ``your_username ALL=(ALL) NOPASSWD: ALL`` at the end of your sudoers file with the command : ``sudo visudo``, then try again.
6. Test the speaker with ``sam test speaker``
7. Test the microphone with ``sam test microphone``
8. If tests are not conclusive or quality is poor, try selecting a different speaker and microphone with : ``sam setup audio``
9. Connect to DEVINE's snips account : ``sam login``, email: devine.gegi@listes.usherbrooke.ca , password is in Devine private repo
10. Download the assistant : ``sam install assistant``
11. Install the requirements : ``sudo -H pip install -r requirements.txt``
12. Run the setup script ``sudo python setup.py install``


Usage
================

.. code-block:: bash

  roscore #start ROS master
  rosrun devine_dialog snips.py #run snips node
  sam watch
  rostopic echo /answer #listen to the answers



To send custom data to the topic used by snips, do :

``rosrun rqt_gui rqt_gui``

- Select topic : /devine/tts/answer
- Select type : devine_dialog/TtsQuery
- Select a frequency
- Fill out the 'text' (ex: "Is the object blue ?"), 'uid' (ex: 1) and 'answer_type' (ex: 1) fields.




.. _SNIPS: https://snips.ai/
