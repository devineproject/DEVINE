import $ from 'jquery';
import ROSLIB from 'roslib';
import LogConsole from './console';

const cons = new LogConsole("ROS", "grey");
const btnReconnect = $("#reconnect_to_ros");
const rosUrl = `ws://${window.location.hostname}:9090`;

export const ros = new ROSLIB.Ros({ url: rosUrl });

export class RosTopic extends ROSLIB.Topic {
  constructor(topic) {
    super({
      ros: ros,
      name: topic.name,
      messageType: topic.type
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
