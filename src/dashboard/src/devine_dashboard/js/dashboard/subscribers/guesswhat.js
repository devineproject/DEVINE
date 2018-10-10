import { RosTopic } from '../ros';
import LogConsole from '../console';
import $ from 'jquery';

const cons = new LogConsole("GuessWhat", "#00bc8c");
const subscriber = $("#guesswhat_checkbox");

export default function InitGuesswhatModule(devineTopics) {
  const topics = {
    state:    new RosTopic(devineTopics.guesswhat_status),
  };
  
  subscriber.on("change", function () {
    $('#guesswhat_ask_btn').prop('disabled', !this.checked);
    if (this.checked) {
      topics.state.subscribe(message => cons.log(`State: ${message.data}`));
  
      cons.log("Subscribed");
    } else {
      for (let i in topics) {
        topics[i].removeAllListeners();
      }
  
      cons.log("Unsubscribed");
    }
  });
}
