import ros from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("Snips", "#F39C12");
const snipsCheckbox = $("#snips_checkbox");

const answer_listener = new ROSLIB.Topic({
  ros: ros,
  latch: true,
  name: '/snips_answer',
  messageType: 'std_msgs/String'
});

const ask_listener = new ROSLIB.Topic({
  ros: ros,
  latch: true,
  name: '/snips_ask',
  messageType: 'std_msgs/String'
});

snipsCheckbox.on("change", function () {
  $('#snips_ask_btn').prop('disabled', !this.checked);
  if (this.checked) {
    answer_listener.subscribe(function (message) {
      if (message.data && message.data.indexOf("|") !== -1) {
        var [receivedMessage, detectedMessage] = message.data.split("|");
        cons.log(`Answer received: ${receivedMessage}`)
        cons.log(`Answer detected: ${detectedMessage}`)
      }
    });
    ask_listener.subscribe(function (message) {
      cons.log(`Question sent: ${message.data}`)
    });
    cons.log("Subscribed");
  } else {
    answer_listener.removeAllListeners();
    ask_listener.removeAllListeners();
    cons.log("Unsubscribed");
  }
});

$("#snips_ask_btn").on("click", function() {
  var val = $("#snips_ask").val() || $("#snips_ask").attr("placeholder");
  ask_listener.publish({data:val});
});


