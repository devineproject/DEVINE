import ros from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("Game State", "#E74C3C");
const gameStateCheckbox = $("#gamestate_checkbox");

const listener = new ROSLIB.Topic({
  ros: ros,
  name: '/game_system_state',
  messageType: 'std_msgs/String'
});

gameStateCheckbox.on("change", function () {
  const view = $('.command-view[name="gamestate"]').find('.subscriber-log')[0];
  if (this.checked) {
    listener.subscribe(function (message) {
      cons.log(message.data);
    });
    cons.log("Subscribed")
  } else {
    listener.removeAllListeners();
    cons.log("Unsubscribed")
  }
});


