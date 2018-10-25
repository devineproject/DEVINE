import { RosTopic } from "../ros";
import LogConsole from "../console";

const cons = new LogConsole("Game State", "#E74C3C");

/**
 * Initialize the module.
 * @param {dict} devineTopics - The list of ros topics.
 */
export default function InitGamestateModule(devineTopics) {
  const topics = {
    category: new RosTopic(devineTopics.guess_category)
  };

  cons.log("Subscribed");
  topics.category.subscribe(message =>
    cons.log(`Game ended! Found the ${message.data}.`)
  );
}
