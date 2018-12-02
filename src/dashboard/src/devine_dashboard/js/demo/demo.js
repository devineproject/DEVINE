import $ from "jquery";
import Dialog from "./dialog";

/**
 * Generate the demo dashboard.
 * @param {dict} devineTopics - The available topics.
 */
export default function CreateDemo(devineTopics) {
  $(document).ready(function() {
    new Dialog(devineTopics);
  });
}
