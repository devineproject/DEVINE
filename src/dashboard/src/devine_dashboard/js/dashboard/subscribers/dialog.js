import LogConsole from "../console";
import QueriesAndAnswers from "./queries_answers";
import $ from "jquery";

/**
 * Initialize the module.
 * @param {dict} devineTopics - The list of ros topics.
 */
export default function initDialogModule(devineTopics) {
  const cons = new LogConsole("Dialog", "#F39C12");

  const queriesAndAnswers = new QueriesAndAnswers(
    devineTopics,
    queryMessage => {
      cons.log(`Querying TTS (${queryMessage.uid}): ${queryMessage.text}`);
    },
    (answerMessage, isQueryAnswered) => {
      if (isQueryAnswered) {
        cons.log(`Answer STT (${answerMessage.original_query.uid}): ${answerMessage.text}`);
      } else {
        cons.log(
          `ERROR: Answer without query for uid ${answerMessage.uid}: ${answerMessage.text}`
        );
      }
    }
  );

  const sendAnswer = () => queriesAndAnswers.publishAnswer($("#dialog_answer").val());
  $("#dialog_publish").on("click", sendAnswer);
  $("#dialog_answer").on("keypress", function(e) {
    if (e.which == 13) {
      sendAnswer();
    }
  });
}
