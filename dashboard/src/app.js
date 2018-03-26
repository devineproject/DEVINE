import _ from 'uikit/dist/css/uikit.min.css';
import ROSLIB from 'roslib';
import $ from 'cash-dom';
import route from './route';
import gamestate from './subscribers/gamestate';
import kinect from './subscribers/kinect';
import snips from './subscribers/snips';

route(window.location.pathname, true);
