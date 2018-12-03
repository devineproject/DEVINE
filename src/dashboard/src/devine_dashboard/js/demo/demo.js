import $ from "jquery";
import Dialog from "./dialog";
import CanvasDrawer from "../dashboard/canvas_drawer";
import { ros, reconnectToRos } from "../dashboard/ros";

/**
 * Generate the demo dashboard.
 * @param {dict} devineTopics - The available topics.
 */
export default function CreateDemo(devineTopics) {
  $(document).ready(function() {
    new Dialog(devineTopics);
    
    const live_feed = new CanvasDrawer(devineTopics, "#live_feed");
    const segmentation_feed = new CanvasDrawer(devineTopics, "#segmentation_feed");
  
    live_feed.changeImageSource('image_raw');
    segmentation_feed.changeImageSource('segmentation_image');
  });

  let countdown;
  function tryAutomaticReconnect() {
    $('#reconnection-modal').modal({ backdrop: 'static'});

    let seconds = 3;
    countdown = setInterval(() => {
      if (seconds === 0) {
        reconnectToRos();
        clearInterval(countdown);
      }

      $('#connection-timeout').text(seconds);
      seconds--;
    }, 1000);
  }

  ros.on("close", () => tryAutomaticReconnect());
  ros.on("connection", () => {
    clearInterval(countdown);
    $('#reconnection-modal').modal('hide');
  });
}
