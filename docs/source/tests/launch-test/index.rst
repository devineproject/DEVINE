Running the unit tests with catkin
##################################

From your catkin workspace run the following:

.. code-block:: bash

    $ catkin_make run_tests

This command will launch all the necessary nodes and run the tests.


Launching a single test case
============================

Each `test_*.py` file corresponds to a test case.

Each one of these files can run individually like so:

.. code-block:: bash

    $ python DEVINE/tests/src/devine_tests/*/test_*.py
