import { RosTopic } from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("Game State", "#E74C3C");
const subscriber = $("#gamestate_checkbox");

const listener = new RosTopic('/game_system_state', 'std_msgs/String');

subscriber.on("change", function () {
  if (this.checked) {
    listener.subscribe((message) => cons.log(message.data));
    cons.log("Subscribed");
  } else {
    listener.removeAllListeners();
    cons.log("Unsubscribed");
  }
});


