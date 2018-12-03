import QueriesAndAnswers from "../dashboard/subscribers/queries_answers";
import { RosTopic } from "../dashboard/ros";
import LogConsole from "../dashboard/console";

/** Control how the dialog will be shown on the demo page */
export default class Dialog {
  constructor(devineTopics) {
    new RosTopic(devineTopics.guesswhat_status).subscribe(this.onGuessWhatStatus);
    new RosTopic(devineTopics.guess_category).subscribe(this.onGuessCategory);
    this.queriesAndAnswers = new QueriesAndAnswers(devineTopics, this.onQuery, this.onAnswer);

    this.devineCons = new LogConsole("DEVINE", "#F39C12");
    this.playerCons = new LogConsole("Player", "#00BC8C");

    this.setupShortcutKeys();
  }

  setupShortcutKeys() {
    document.onkeypress = event => {
      if (event.keyCode === 121) {
        // Y key pressed
        this.queriesAndAnswers.publishAnswer("yes");
      } else if (event.keyCode === 110) {
        // N key pressed
        this.queriesAndAnswers.publishAnswer("no");
      }
    };
  }

  onQuery(message) {
    this.devineCons.log(message.text);
  }

  onAnswer(message, isQueryAnswered) {
    this.playerCons.log(isQueryAnswered
      ? message.text
      : `Message without any query: ${message.text}`);
  }

  onGuessCategory(message) {
    this.devineCons.log(`Game ended! Found the ${message.data}.`);
  }

  onGuessWhatStatus(message) {
    this.devineCons.log(`GuessWhat state: ${message.data}`);
  }
}
