import { RosTopic } from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("Snips", "#F39C12");
const subscriber = $("#snips_checkbox");

const topics = {
  detected_answer: new RosTopic('/answer', 'std_msgs/String'),
};

subscriber.on("change", function () {
  $('#snips_ask_btn').prop('disabled', !this.checked);

  if (this.checked) {
    topics.detected_answer.subscribe(message => cons.log(`Answer: ${message.data}`));

    cons.log("Subscribed");
  } else {
    for (let i in topics) {
      topics[i].removeAllListeners();
    }

    cons.log("Unsubscribed");
  }
});
