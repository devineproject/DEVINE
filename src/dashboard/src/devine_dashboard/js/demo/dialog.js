import $ from "jquery";
import QueriesAndAnswers from "../dashboard/subscribers/queries_answers";
import { RosTopic } from "../dashboard/ros";
import LogConsole from "../dashboard/console";

/** Control how the dialog will be shown on the demo page */
export default class Dialog {
  constructor(devineTopics) {
    new RosTopic(devineTopics.guesswhat_status).subscribe(m => this.onGuessWhatStatus(m));
    new RosTopic(devineTopics.guess_category).subscribe(m => this.onGuessCategory(m));
    this.queriesAndAnswers = new QueriesAndAnswers(devineTopics, m => this.onQuery(m), (m, q) => this.onAnswer(m, q));

    this.devineCons = new LogConsole("DEVINE", "#F39C12");
    this.playerCons = new LogConsole("Player", "#00BC8C");

    this.setupShortcutKeys();
  }

  /** Hook to html elements for user's answer shortcuts. */
  setupShortcutKeys() {
    const sendYes = () => this.queriesAndAnswers.publishAnswer("yes");
    const sendNo = () => this.queriesAndAnswers.publishAnswer("no");
    const sendAnswer = () => {
      if ($("#dialog_answer").val() != "") {
        this.queriesAndAnswers.publishAnswer($("#dialog_answer").val());
        $("#dialog_answer").val("");
      }
    };

    document.onkeypress = event => {
      if ($(event.target).is('input, textarea')) {
        return;   
      }
      if (event.keyCode === 121) {
        sendYes(); // on 'Y' key pressed
      } else if (event.keyCode === 110) {
        sendNo(); // on 'N' key pressed
      }
    };

    $("#dialog_publish_yes").on("click", sendYes);
    $("#dialog_publish_no").on("click", sendNo);

    $("#dialog_publish").on("click", sendAnswer);
    $("#dialog_answer").on("keypress", function(e) {
      if (e.which == 13) {
        sendAnswer();
      }
    });
  }

  /**
   * Callback when a query message is received.
   * @param {object} message - The reply.
   */
  onQuery(message) {
    this.devineCons.log(message.text);
  }

  /**
   * Callback when a answer message is received.
   * @param {object} message - The reply.
   * @param {boolean} isQueryAnswered - Indicate if this reply has answered a query.
   */
  onAnswer(message, isQueryAnswered) {
    this.playerCons.log(isQueryAnswered
      ? message.text
      : `Message without any query: ${message.text}`);
  }

  /**
   * Callback when a guess category message is received.
   * @param {object} message - The reply.
   */
  onGuessCategory(message) {
    this.devineCons.log(`Game ended! Found the ${message.data}.`);
  }

  /**
   * Callback when a guesswhat status message is received.
   * @param {object} message - The reply.
   */
  onGuessWhatStatus(message) {
    this.devineCons.log(`GuessWhat state: ${message.data}`);
  }
}
