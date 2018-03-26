import ros from '../ros';
import ROSLIB from 'roslib';
import $ from 'cash-dom';

const answer_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/snips_answer',
  messageType: 'std_msgs/String'
});

const ask_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/snips_ask',
  messageType: 'std_msgs/String'
});

$('.command-view[name="snips"]').find('input[type="checkbox"]').on("change", function () {
  const view = $('.command-view[name="snips"]').find('.subscriber-log')[0];
  if (this.checked) {
    answer_listener.subscribe(function (message) {
      if (message.data && message.data.indexOf("|") !== -1) {
        var [receivedMessage, detectedMessage] = message.data.split("|");
        view.innerText += `Answer received: ${receivedMessage}\n`;
        view.innerText += `Answer detected: ${detectedMessage}\n`;
        view.scrollTop = view.scrollHeight;
      }
    });
    ask_listener.subscribe(function (message) {
      view.innerText += `Question sent: ${message.data}\n`;
      view.scrollTop = view.scrollHeight;
    });
  } else {
    answer_listener.unsubscribe();
    ask_listener.unsubscribe();
    view.innerText += '=======Unsubscribed=======\n';
    view.scrollTop = view.scrollHeight;
  }
});

$("#snips_ask_btn").on("click", function() {
  var val = $("#snips_ask").val();
  ask_listener.publish({data:val});
});


