import { RosTopic } from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("Game State", "#E74C3C");
const subscriber = $("#gamestate_checkbox");

const topics = {
  category: new RosTopic('/found_category', 'std_msgs/String'),
};

subscriber.on("change", function () {
  if (this.checked) {
    topics.category.subscribe(message => cons.log(`Game ended! Found the ${message.data}.`));
    
    cons.log("Subscribed");
  } else {
    for (let i in topics) {
      topics[i].removeAllListeners();
    }

    cons.log("Unsubscribed");
  }
});


