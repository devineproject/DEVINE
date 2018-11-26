import { RosTopic } from "../ros";
import ROSLIB from "roslib";
import LogConsole from "../console";
import $ from "jquery";

/**
 * Initialize the module.
 * @param {dict} devineTopics - The list of ros topics.
 */
export default function initDialogModule(devineTopics) {
  const cons = new LogConsole("Dialog", "#F39C12");
  const answerField = $("#dialog_answer");

  const topics = {
    ttsQuery: new RosTopic(devineTopics.tts_query),
    ttsAnswer: new RosTopic(devineTopics.tts_answer)
  };

  const answer_types = {
    NO_ANSWER: 0,
    YES_NO: 1,
    PLAYER_NAME: 2
  };

  let queries = [];
  function publish() {
    const answer = answerField.val();
    if (answer !== "") {
      let query_answered = false;
      for (let i = queries.length - 1; i >= 0; --i) {
        const query = queries[i];
        if (query.answered) {
          continue;
        }

        new RosTopic(devineTopics.tts_answer).publish(
          new ROSLIB.Message({
            original_query: query,
            probability: 1.0,
            text: answer
          })
        );
        
        answerField.val("");
        query.answered = true;
        query_answered = true;
      }

      if (!query_answered) {
        cons.log("No TTS query to answer");
      }
    }
  }

  $("#dialog_publish").on("click", publish);
  $("#dialog_answer").on("keypress", function(e) {
    if (e.which == 13) {
      publish();
    }
  });

  cons.log("Subscribed");

  topics.ttsQuery.subscribe(message => {
    if (message.answer_type !== answer_types.NO_ANSWER) {
      queries.push(message);
    }
    cons.log(`Querying TTS (${message.uid}): ${message.text}`);
  });

  topics.ttsAnswer.subscribe(message => {
    let query_answered = false;
    for (let i in queries) {
      if (queries[i].uid == message.original_query.uid) {
        queries = queries.splice(i, 1);
        query_answered = true;
        cons.log(`Answer STT (${message.uid}): ${message.text}`);
        break;
      }
    }

    if (!query_answered) {
      cons.log(
        `ERROR: Answer without query for uid ${message.uid}: ${message.text}`
      );
    }
  });
}
