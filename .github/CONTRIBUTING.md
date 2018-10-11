Contributing to DEVINE
=====================

## Javascript coding standard
We use ESLint to enforce coding standards.
ESLint can be installed using npm:
```bash
sudo npm i -g eslint
```

Once installed, you can validate our code standard on any javascript file by typing:

```bash
eslint -c PATH_TO_REPO/.eslintrc.json JS_FILE.js
```

Or validate every file on your favorite IDE by using one of the integration available in [the documentation of ESLint](https://eslint.org/docs/user-guide/integrations).

## Python coding standard

DEVINE uses [Pylint](https://www.pylint.org/) to enforce coding standards for Python. Additionnaly a git pre-commit hook for Pylint is also used to run the linter before all commits and ensure higher code quality. The commit will be blocked if the quality of the **.py** files don't pass a certain threshold.

### Installation

#### 1. Pylint

```
$ pip install pylint
```

Once installed, you can run Pylint manually on any **.py** file by typing:

```
$ pylint MY_PYTHON_FILE.py --rcfile=PATH_TO_DEVINE_REPO/.pylintrc
```

Note that Pylint can be integrated into various IDE as seen  [here](https://pylint.readthedocs.io/en/latest/user_guide/ide-integration.html).

#### 2. git-pylint-commit-hook 

```
pip install git-pylint-commit-hook
```

Navigate to your .git/hooks/ directory. From the root directory of your project, run the following command to rename the pre-commit.sample file to pre-commit

```
cd .git/hooks/
mv pre-commit.sample pre-commit
```

Delete everything thats in there and paste this in the pre-commit file

```
#!/bin/sh
git-pylint-commit-hook --pylintrc PATH_TO_DEVINE_REPO/.pylintrc --limit 8.0
```

### Understand Pylint outputs

Once Pylint is done analyzing the **.py** files, it can output 5 different types of messages


  * (C) convention, for programming standard violation
  * (R) refactor, for bad code smell
  * (W) warning, for python specific problems
  * (E) error, for much probably bugs in the code
  * (F) fatal, if an error occurred which prevented pylint from doing

Sometimes the default output for the messages need a bit more info. Lets use this line as an exemple:

```
"C:  1: Missing docstring (missing-docstring)"
```

Any time you don't understand a message, you can get more information on it with the **--help-msg** command. Following the exemple line of code above, you can type:

```
$ pylint --help-msg=missing-docstring
```

More on it [here](https://pylint.readthedocs.io/en/latest/tutorial.html)
