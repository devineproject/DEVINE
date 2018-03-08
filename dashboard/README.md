# 🤖 Robot dashboard

Install dependencies

```
npm install
pip install -r requirements.txt
```

Build

```
webpack
```

Run

```
roslaunch rosbridge_server rosbridge_websocket.launch
```

```
python serve.py
```

## Adding a view

Create an html layout for your view (Uikit is your friend) e.g `views/myview.html`. Or reuse one similar to yours.

`include` it in `views/index.html`, keep these class attributes `uk-width-expand command-view` and change the name attribute.

```html
<div class="uk-width-expand command-view" name="myview" hidden>
  {% include 'myview.html' %}
</div>
```

Add it to the menu with a class attribute matching the name you used previously.

```html
<li class="command-myview command-menu">My view</li>
```

Code your view in its own file (`src/myview.js`) and `import` it in `src/app.js`.
