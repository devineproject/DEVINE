About the tests
###############

The tests are made using `Python unittest <https://docs.python.org/3/library/unittest.html>`_.

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


Adding test cases
#################

To add a test case, simply copy the `testcase_template.py` into your test folder, then import your test case into `test_suite.py`.
