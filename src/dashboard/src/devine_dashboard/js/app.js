import ROSLIB from 'roslib';
import $ from 'jquery';
import dashboard from "./dashboard/dashboard";
import scoreboard from "./scoreboard/scoreboard";

try {
  // Fix popper.js for Bootstrap4
  window.$ = window.jQuery = require('jquery');
  window.Popper = require('popper.js').default;
  require('bootstrap');
} catch (e) { console.error(e); }

switch (window.route) {
case "scoreboard":
  scoreboard();
  break;
case "index": //Passthrough as default
default:
  dashboard();
  break;
}