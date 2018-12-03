import LogConsole from "../console";
import CanvasDrawer from "../canvas_drawer";
import $ from "jquery";

// Variables definition
const cons = new LogConsole("Kinect", "#3498DB");
const objectPos2d = $("#kinect_pos_found")[0];
const objectPos3d = $("#kinect_pos_calc")[0];
const delay = $("#kinect_image_delay");
const image_selection = $("input[name=image_selection]");

/**
 * Initialize the module.
 * @param {dict} devineTopics - The list of ros topics.
 */
export default function InitKinectModule(devineTopics) {
  const canvasDrawer = new CanvasDrawer(
    devineTopics,
    "#kinect_image",
    writePositions,
    len => delay.prop('max', len)
  );

  /** Handle image changed */
  function imageSourceChanged() {
    if ($(this).is(":checked")) {
      let currentImgType = $(this).val();
      canvasDrawer.changeImageSource(currentImgType);
      cons.log(`Subscribed to ${currentImgType}`);
    }
  }
  image_selection.change(imageSourceChanged);
  image_selection.each(imageSourceChanged); //Initialisation

  delay.on("change", function() {
    canvasDrawer.setHistoryPosition(Math.max(this.value, 1));
  });

  /**
   * Write positions on the dashboard.
   * @param {array} obj_pos_2d - The 2D position.
   * @param {array} obj_pos_3d - The 3D position.
   */
  function writePositions(obj_pos_2d, obj_pos_3d) {
    if (obj_pos_2d != undefined) {
      objectPos2d.innerText = `(${obj_pos_2d.point.x}, ${obj_pos_2d.point.y})`;
    }

    if (obj_pos_3d != undefined) {
      let cleanFloat = number => (number === null ? "N/A" : number.toFixed(2));

      objectPos3d.innerText =
        `(${cleanFloat(obj_pos_3d.pose.position.x)}, ` +
        `${cleanFloat(obj_pos_3d.pose.position.y)}, ${cleanFloat(obj_pos_3d.pose.position.z)})`;
    }
  }
}
