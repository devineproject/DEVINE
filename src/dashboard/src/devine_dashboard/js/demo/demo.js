import $ from "jquery";
import Dialog from "./dialog";
import CanvasDrawer from "../dashboard/canvas_drawer";

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
}
