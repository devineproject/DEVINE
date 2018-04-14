import ROSLIB from 'roslib';
// import route from './route';
import gamestate from './subscribers/gamestate';
import kinect from './subscribers/kinect';
import snips from './subscribers/snips';
import $ from 'jquery'

try {
    // Fix popper.js for Bootstrap4
    window.$ = window.jQuery = require('jquery');
    window.Popper = require('popper.js').default;
    require('bootstrap');
} catch (e) {}

$(document).ready(function () {
    // Uncheck all subscriptions
    $('[type="checkbox"]').prop('checked', false);
})

// route(window.location.pathname, true);
