import ROSLIB from 'roslib';
import { devineTopics } from './vars'
import LogConsole from './console'
import $ from 'jquery';

const cons = new LogConsole("ROS", "grey")
const btnReconnect = $("#reconnect_to_ros");
const rosUrl = `ws://${window.location.hostname}:9090`;

export const ros = new ROSLIB.Ros({ url: rosUrl });

export class RosTopic extends ROSLIB.Topic {
  constructor(name, messageType) {
    super({
      ros: ros,
      name: name,
      messageType: messageType
    });
  }
}

function logAndShowReconnect(message) {
  cons.log(message);
  btnReconnect.show();
}

ros.on('connection', () => cons.log('Rosbridge connection established'));
ros.on('error', () => logAndShowReconnect('Rosbridge connection error'));
ros.on('close', () => logAndShowReconnect('Rosbridge connection closed'));

btnReconnect.on("click", function () {
  ros.connect(rosUrl);
  btnReconnect.hide();
});
