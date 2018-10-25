import $ from "jquery";
import publisher from "./publisher";
import gamestate from "./subscribers/gamestate";
import kinect from "./subscribers/kinect";
import dialog from "./subscribers/dialog";
import guesswhat from "./subscribers/guesswhat";
import imgdispatcher from "./subscribers/image_dispatcher";

/**
 * Generate the dashboard.
 * @param {dict} rosTopics - The available topics.
 */
export default function CreateDashboard(rosTopics) {
  $(document).ready(function() {
    // Uncheck all subscriptions
    $('[type="checkbox"]').prop("checked", false);

    publisher(rosTopics);
    kinect(rosTopics);
    dialog(rosTopics);
    gamestate(rosTopics);
    guesswhat(rosTopics);
    imgdispatcher(rosTopics);
  });
}
