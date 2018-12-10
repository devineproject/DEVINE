import { RosTopic } from "../ros";
import ROSLIB from "roslib";

const ANSWER_TYPES = {
  NO_ANSWER: 0,
  YES_NO: 1,
  PLAYER_NAME: 2
};

export default class QueriesAndAnswers {
  /**
   * Control how the dialog will be shown on the demo page.
   * @param {dict} devineTopics - The list of ros topics.
   * @param {func} queryCallback - Callback when a query is received.
   * @param {func} answerCallBack - Callback when an answer is received.
   */
  constructor(devineTopics, queryCallback, answerCallBack) {
    this.topics = {
      ttsQuery: new RosTopic(devineTopics.tts_query),
      ttsAnswer: new RosTopic(devineTopics.tts_answer)
    };

    this.topics.ttsQuery.subscribe(m => this.onQueryReceived(m));
    this.topics.ttsAnswer.subscribe(m => this.onAnswerReceived(m));

    this.queries = [];
    this.queryCallback = queryCallback;
    this.answerCallBack = answerCallBack;
  }

  /**
   * Control how the dialog will be shown on the demo page.
   * @param {string} text - The text to send.
   * @return {boolean} True if query is answered.
   */
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
      break;
    }

    return query_answered;
  }

  /**
   * Callback when a query message is received.
   * @param {object} message - The reply.
   */
  onQueryReceived(message) {
    if (message.answer_type !== ANSWER_TYPES.NO_ANSWER) {
      this.queries.push(message);
    }
    this.queryCallback(message);
  }

  /**
   * Callback when a answer message is received.
   * @param {object} message - The reply.
   */
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
