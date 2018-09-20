##############################
Welcome to the DEVINE Project!
##############################

The goal of the DEVINE Project is to serve as a demonstration on how research results in grounded language learning and understanding can be used in a cooperative task between an intelligent agent and a human.
The task, undertaken by a robot, is the question answering game GuessWhat?!. The robot interacts with a human player who performs the role of the oracle.
The oracle selects an object in a visual scene without disclosing it to the robot.
Playing the role of the guesser, the robot asks questions to the person to deduce which object was selected.
The guesser’s questions are generated using the player’s previous answers and visual information.
Once the robot has a strong belief, it guesses the object chosen by the player and physically points to it in the scene.

The robotic platform for the demonstration is IRL-1. By design, it can operate in a typical human environment.
It is equipped with a Kinect, differential elastic actuators for its arms, a microphone array, and powered wheels with omni directional capability.
As IRL-1 has compliant joints, players are protected from physical harm.
To begin playing GuessWhat?! with IRL-1, a player approaches the robot.
Once the player is close enough, IRL-1 asks if they want to play.
After explaining the rules, IRL-1 takes an image of the scene comprising different objects.
Then, it turns back to the player and asks if they have selected an object.
Upon confirmation, IRL-1 begins to ask them questions.
After five questions, the robot reveals its guess.
The player validates IRL-1’s guess, IRL-1 reacts accordingly and then concludes the game.

***************************
Accessing the documentation
***************************

If you are on linux with Docker installed, you directly run this command to launch the DEVINE's documentation:

.. code-block:: bash
    
    ./load-documentation.sh

Please follow the following link for all the documentation goodness:

`Documentation`_.

.. _Documentation: /docs/source/index.rst
