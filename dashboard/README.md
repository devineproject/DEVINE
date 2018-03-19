# 🤖 Robot dashboard

Install dependencies

```bash
sudo npm i -g webpack
npm install
pip3 install -r requirements.txt
sudo apt-get install ros-kinetic-rosbridge-server
```

Build

```bash
npm run build
```

Run

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

```bash
npm start
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
