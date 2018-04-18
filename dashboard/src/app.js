import ROSLIB from 'roslib';
import publisher from './publisher';
import gamestate from './subscribers/gamestate';
import kinect from './subscribers/kinect';
import snips from './subscribers/snips';
import guesswhat from './subscribers/guesswhat';
import $ from 'jquery'

try {
  // Fix popper.js for Bootstrap4
  window.$ = window.jQuery = require('jquery');
  window.Popper = require('popper.js').default;
  require('bootstrap');
} catch (e) { console.error(e); }

$(document).ready(function () {
  // Uncheck all subscriptions
  $('[type="checkbox"]').prop('checked', false);
})
