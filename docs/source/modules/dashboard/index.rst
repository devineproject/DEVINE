Dashboard
#########

Description
===========

The dashboard is a web based project where we integrate all of the ROS nodes and gives us a centralized operation center.
You can subscribe to any ROS topic and see what is being send on any topic and you can also send information to them.
It's main goal is to allow us to verify that the whole DEVINE system works in harmony.

It can also be used to demo the project.


Usage
=====

Once the project is installed on your machine, you can simply launch the dashboard like so:

.. code-block:: bash

    $ roslaunch devine devine.launch launch_all:=false dashboard:=true

The process will listen and update whenever there is a change in the code.


Manual installation
===================

.. code-block:: bash

    $ sudo npm i -g webpack
    $ npm install
    $ pip3 install -r requirements.txt
    $ sudo apt-get install ros-kinetic-rosbridge-server


Adding a view
=============

Create an html layout for your view. E.g: `views/myview.html`. Or reuse one similar to yours.

`include` it in `views/index.html`, keep these class attributes `uk-width-expand` `command-view` and change the name attribute.

.. code-block:: html

    <div class="uk-width-expand command-view" name="myview" hidden>
        {% include 'myview.html' %}
    </div>

Add it to the menu with a class attribute matching the name you used previously.

.. code-block:: html

    <li class="command-myview command-menu">My view</li>

Code your view in its own file (`src/myview.js`) and import it in `src/app.js`.
