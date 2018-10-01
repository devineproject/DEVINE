import { RosTopic } from '../ros';
import devineTopics from '../../vars/devine_topics.json';
import LogConsole from '../console';
import $ from 'jquery';

export default function()
{
  const cons = new LogConsole("Dialog", "#F39C12");
  const subscriber = $("#dialog_checkbox");

  const topics = {
    ttsQuery: new RosTopic(devineTopics.tts_query),
    ttsAnswer: new RosTopic(devineTopics.tts_answer)
  };

  let queries = [];

  subscriber.on("change", function () {
    $('#dialog_ask_btn').prop('disabled', !this.checked);
  
    if (this.checked) {
      cons.log("Subscribed");
      
      topics.ttsQuery.subscribe(message => {
        if (message.answer_type !== 0) {
          queries.push(message);
        }
        cons.log(`Querying TTS (${message.uid}): ${message.text}`);
      });
      topics.ttsAnswer.subscribe(message => {
        let query_answered = false;
        for (let i in queries) {
          if (queries[i].uid == message.uid) {
            queries = queries.splice(i, 1);
            query_answered = true;
            cons.log(`Answer STT (${message.uid}): ${message.text}`);
            break;
          }
        }
        if (!query_answered) {
          cons.log(`ERROR: Answer without query for uid ${message.uid}: ${message.text}`);
        }
      });
    } else {
      for (let i in topics) {
        topics[i].removeAllListeners();
      }
      cons.log("Unsubscribed");
    }
  });
}


