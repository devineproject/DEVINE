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
