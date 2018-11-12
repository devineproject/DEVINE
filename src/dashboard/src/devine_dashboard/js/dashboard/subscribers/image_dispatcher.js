import { RosTopic } from "../ros";
import $ from "jquery";

/**
 * Initialize the module.
 * @param {dict} devineTopics - The list of ros topics.
 */
export default function InitImgDispatcherModule(devineTopics) {
  const topics = {
    segmentation_image: new RosTopic(devineTopics.segmentation_image),
    body_tracking_image: new RosTopic(devineTopics.body_tracking_image),
    features_image: new RosTopic(devineTopics.features_extraction_image)
  };

  /**
   * Resend an image to update the dashboard on user action.
   * @param {Topic} topic - The topic where to publish.
   */
  let republish_from_img = function(topic) {
    return function() {
      const image_topic = new RosTopic(devineTopics.compressed_image);
      image_topic.subscribe(img => {
        topic.publish(img);
        image_topic.unsubscribe();
        image_topic.removeAllListeners();
      });
    };
  };

  $("#dispatch_segm").click(republish_from_img(topics.segmentation_image));
  $("#dispatch_fea").click(republish_from_img(topics.features_image));
  $("#dispatch_body").click(republish_from_img(topics.body_tracking_image));
}
