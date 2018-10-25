import { RosTopic } from "../ros";
import LogConsole from "../console";
import $ from "jquery";

const cons = new LogConsole("GuessWhat", "#00bc8c");

export default function InitGuesswhatModule(devineTopics) {
  const topics = {
    state: new RosTopic(devineTopics.guesswhat_status)
  };

  cons.log("Subscribed");
  topics.state.subscribe(message => cons.log(`State: ${message.data}`));
}
