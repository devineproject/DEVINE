import $ from "jquery";
import ROSLIB from "roslib";
import LogConsole from "./console";

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

/** Try to reconnect to ROS */
export function reconnectToRos() {
  ros.connect(rosUrl);
  btnReconnect.hide();
}

/**
 * Log in the console the reason why the connection to ros has been lost.
 * Also show the reconnect button.
 * @param {string} message - The message to log.
 */
function logErrorAndShowReconnect(message) {
  console.log(message);
  btnReconnect.show();
}

ros.on("connection", () => cons.log("Rosbridge connection established"));
ros.on("error", () => logErrorAndShowReconnect("Rosbridge connection error"));
ros.on("close", () => logErrorAndShowReconnect("Rosbridge connection closed"));

btnReconnect.on("click", reconnectToRos);
