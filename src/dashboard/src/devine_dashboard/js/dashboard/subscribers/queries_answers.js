import { RosTopic } from "../ros";
import ROSLIB from "roslib";

const ANSWER_TYPES = {
  NO_ANSWER: 0,
  YES_NO: 1,
  PLAYER_NAME: 2
};

/** Control how the dialog will be shown on the demo page */
export default class QueriesAndAnswers {
  constructor(devineTopics, queryCallback, answerCallBack) {
    this.topics = {
      ttsQuery: new RosTopic(devineTopics.tts_query),
      ttsAnswer: new RosTopic(devineTopics.tts_answer)
    };

    this.topics.ttsQuery.subscribe(this.onQueryReceived);
    this.topics.ttsAnswer.subscribe(this.onAnswerReceived);

    this.queries = [];
    this.queryCallback = queryCallback;
    this.answerCallBack = answerCallBack;
  }

  /** Return true if query is answered */
  publishAnswer(text) {
    if (text === "") {
      return false;
    }

    let query_answered = false;
    for (let i = this.queries.length - 1; i >= 0; --i) {
      const query = this.queries[i];
      if (query.answered) {
        continue;
      }

      this.topics.ttsAnswer.publish(
        new ROSLIB.Message({
          original_query: query,
          probability: 1.0,
          text: text
        })
      );
      
      query.answered = true;
      query_answered = true;
    }

    return query_answered;
  }

  onQueryReceived(message) {
    if (message.answer_type !== ANSWER_TYPES.NO_ANSWER) {
      this.queries.push(message);
    }
    this.queryCallback(message);
  }

  onAnswerReceived(message) {
    let isQueryAnswered = false;
    for (let i in this.queries) {
      if (this.queries[i].uid == message.original_query.uid) {
        this.queries = this.queries.splice(i, 1);
        isQueryAnswered = true;
        break;
      }
    }

    this.answerCallBack(message, isQueryAnswered);
  }
}
