import { RosTopic } from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("Snips", "#F39C12");
const subscriber = $("#snips_checkbox");

const topics = {
  ask: new RosTopic('/question', 'std_msgs/String'),
  answer: new RosTopic('/answer', 'std_msgs/String'),
};

subscriber.on("change", function () {
  $('#snips_ask_btn').prop('disabled', !this.checked);

  if (this.checked) {
    topics.ask.subscribe(message => cons.log(`Question sent: ${message.data}`));
    topics.answer.subscribe(message => {
      if (message.data && message.data.indexOf("|") !== -1) {
        var [receivedMessage, detectedMessage] = message.data.split("|");
        cons.log(`Answer received: ${receivedMessage}`);
        cons.log(`Answer detected: ${detectedMessage}`);
      }
    });

    cons.log("Subscribed");
  } else {
    for (let i in topics) {
      topics[i].removeAllListeners();
    }

    cons.log("Unsubscribed");
  }
});

function ask() {
  var askText = $("#snips_ask");
  topics.ask.publish({ data: (askText.val() || askText.attr("placeholder")) });
}

$("#snips_ask_btn").on("click", ask);
$('#snips_ask').keypress(function (key) {
  if (key.which == 13) {
    ask();
  }
});
