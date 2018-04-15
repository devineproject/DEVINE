import ros from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("GuessWhat", "#00bc8c");
const guesswhatCheckbox = $("#guesswhat_checkbox");

const question_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/question',
  messageType: 'std_msgs/String'
});

const answer_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/answer',
  messageType: 'std_msgs/String'
});

guesswhatCheckbox.on("change", function () {
  $('#guesswhat_ask_btn').prop('disabled', !this.checked);
  if (this.checked) {
    answer_listener.subscribe(function (message) {
      cons.log(`Answer: ${message.data}`)
    });
    question_listener.subscribe(function (message) {
      cons.log(`Question: ${message.data}`)
    });
    cons.log("Subscribed");
  } else {
    answer_listener.removeAllListeners();
    question_listener.removeAllListeners();
    cons.log("Unsubscribed");
  }
});

