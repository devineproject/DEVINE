import { RosTopic } from "../ros";
import LogConsole from "../console";

const cons = new LogConsole("GuessWhat", "#00bc8c");

/**
 * Initialize the module.
 * @param {dict} devineTopics - The list of ros topics.
 */
export default function InitGuesswhatModule(devineTopics) {
  const topics = {
    state: new RosTopic(devineTopics.guesswhat_status)
  };

  cons.log("Subscribed");
  topics.state.subscribe(message => cons.log(`State: ${message.data}`));
}
