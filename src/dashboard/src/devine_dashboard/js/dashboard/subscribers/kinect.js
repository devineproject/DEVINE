import { RosTopic } from "../ros";
import { distinctColors } from "../../vars/colors";
import throttle from "../../throttle";
import LogConsole from "../console";
import $ from "jquery";

// Variables definition
const cons = new LogConsole("Kinect", "#3498DB");
const imageSize = { x: 640, y: 480 };
const objectPos2d = $("#kinect_pos_found")[0];
const objectPos3d = $("#kinect_pos_calc")[0];
const canvas = $("#kinect_image")[0];
const image = canvas ? canvas.getContext("2d") : undefined;
const delay = $("#kinect_image_delay");
const image_selection = $("input[name=image_selection]");

/**
 * Initialize the module.
 * @param {dict} devineTopics - The list of ros topics.
 */
export default function InitKinectModule(devineTopics) {
  const topics = {
    image_raw: new RosTopic(devineTopics.compressed_image),
    segmentation_image: new RosTopic(devineTopics.segmentation_image),
    body_tracking_image: new RosTopic(devineTopics.body_tracking_image),
    zone_detection_image: new RosTopic(devineTopics.zone_detection_image_out),
    confidence: new RosTopic(devineTopics.objects_confidence),
    segmentation: new RosTopic(devineTopics.objects),
    object_position_2d: new RosTopic(devineTopics.guess_location_image),
    object_position_3d: new RosTopic(devineTopics.guess_location_world),
    body_tracking: new RosTopic(devineTopics.body_tracking)
  };

  let history = {
    position: 1,
    image: []
  };

  /* Subscribe to every topics */
  Object.keys(topics).forEach(topic => {
    history[topic] = [];
    subscribeAndBind(topics[topic], history[topic]);
  });

  /** Handle image changed */
  function imageSourceChanged() {
    if ($(this).is(":checked")) {
      let currentImgType = $(this).val();
      history.image = history[currentImgType];
      cons.log(`Subscribed to ${currentImgType}`);
    }
  }
  image_selection.change(imageSourceChanged);
  image_selection.each(imageSourceChanged); //Initialisation

  const draw = throttle(function draw() {
    let img = getCurrentHistoryElement(history.image);
    let stamp = img ? img.header.stamp : null;
    let obj_pos_2d = getCurrentHistoryElement(history.object_position_2d, stamp);
    let obj_pos_3d = getCurrentHistoryElement(history.object_position_3d, stamp);
    let body_tracking = getCurrentHistoryElement(history.body_tracking);
    let confidence = getCurrentHistoryElement(history.confidence);
    let seg = getCurrentHistoryElement(history.segmentation, stamp);
    let image_length = history.image.length;
    
    writePositions(obj_pos_2d, obj_pos_3d);

    if (img != undefined) {
      delay.prop("max", image_length);
      let imageObject = new Image();

      imageObject.onload = function() {
        resetImage(image, imageObject);

        if (obj_pos_2d != undefined) {
          drawPositionFound(obj_pos_2d.point.x, imageSize.y - obj_pos_2d.point.y);
        }

        if (seg != undefined) {
          drawObjectsRectangles(seg.objects, confidence.data);
        }

        if (body_tracking != undefined) {
          drawBodyTracking(JSON.parse(body_tracking.data));
        }
      };
      imageObject.src = "data:image/" + img.format + ";base64, " + img.data;
    }
  }, 100);

  $("#kinect_image_type").on("change", function() {
    topics.image.name = this.value;
  });

  delay.on("change", function() {
    history.position = Math.max(this.value, 1);
    draw();
  });
  drawNoFeed(); // No feed at startup


  /**
   * Set the image canvas color brush for further drawing.
   * @param {string} color - The new color.
   */
  function setColor(color) {
    if (color) {
      image.strokeStyle = color;
      image.fillStyle = color;
    }
  }

  /** Draw an old-school style ne camera feed warning */
  function drawNoFeed() {
    setColor("red");
    image.font = "bold 20pt Arial";
    image.fillText("< No Camera Feed />", 190, imageSize.y / 2);
  }

  /**
   * Draw a cross an the position found
   * @param {int} x - X coordinate.
   * @param {int} y - Y coordinate.
   */
  function drawPositionFound(x, y) {
    setColor("#3498DB");
    image.fillText("Found!", x + 8, y - 8);
    image.fillRect(x - 3, y - 3, 6, 6);
    image.fillRect(x - 20, y - 1, 40, 2);
    image.fillRect(x - 1, y - 20, 2, 40);
  }

  /**
   * Draw rectangle over object found on the image.
   * @param {array} objects - The list of objects.
   * @param {int|undefined} confidence - The confidence value, if an object was found.
   */
  function drawObjectsRectangles(objects, confidence) {
    for (var i = 0; i < objects.length; i++) {
      setColor(distinctColors[i % 14]);

      image.beginPath();
      let {x_offset, y_offset, height, width} = objects[i].bounding_box;

      image.rect(x_offset, y_offset, width, height);

      image.fillText(objects[i].category_name, x_offset, y_offset - 1);

      // Show the confidence about the object found, if any.
      if (confidence !== undefined && confidence.length === objects.length) {
        let obj_name_offset = image.measureText(objects[i].category_name).width;
        image.fillText(`(${(confidence[i]*100).toFixed(2)}%)`, x_offset + obj_name_offset + 5, y_offset - 1);
      }

      image.stroke();
      image.closePath();
    }
  }

  /**
   * Retreive the current history element with the current position.
   * @param {array} historyArray - The array to take the element from.
   * @return {object} The element.
   */
  function getCurrentHistoryElement(historyArray, stamp = null) {
    if (!stamp) {
      // Return latest element if there is no stamp
      return historyArray[historyArray.length - history.position];
    }
    for (let i = historyArray.length - 1; i >= 0; --i) {
      if (Math.abs(historyArray[i].header.stamp.secs - stamp.secs) < 2) {
        return historyArray[i];
      }
    }
  }

  /**
   * Clear the canvas and reset the image.
   * @param {canvas} image - The image canvas.
   * @param {object} imageObject - The image data.
   */
  function resetImage(image, imageObject) {
    image.clearRect(0, 0, imageSize.x, imageSize.y);
    image.drawImage(imageObject, 0, 0);
    image.font = "bold 12pt Arial";
    image.lineWidth = "2";
  }

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

  /**
   * Map body part with one another (e.g.: ear with eye).
   * Adaptation and refactor of function draw_humans in tf_pose/estimator.py
   * @param {array} humans - The list of humans detected.
   *
   * @see CocoPairs in tf_pose/common.py
   */
  function drawBodyTracking(humans) {
    const BodyPartsAggr = [
      [1, 2],
      [1, 5],
      [2, 3],
      [3, 4],
      [5, 6],
      [6, 7],
      [1, 8],
      [8, 9],
      [9, 10],
      [1, 11],
      [11, 12],
      [12, 13],
      [1, 0],
      [0, 14],
      [14, 16],
      [0, 15],
      [15, 17]
    ];
    const leftEyeId = 15,
      rightEyeId = 14;
    for (let i in humans) {
      let centers = {};
      let human = humans[i];
      //For each body parts, find its pixel location in the image
      for (let j in human.body_parts) {
        let bp = human.body_parts[j];
        centers[bp.index] = {
          x: bp.x * imageSize.x + 0.5,
          y: bp.y * imageSize.y + 0.5
        };
      }

      //For each body parts that should be linked, draw a line between these two
      for (let j in BodyPartsAggr) {
        let pair = BodyPartsAggr[j];
        if (!centers[pair[0]] || !centers[pair[1]]) {
          continue;
        }
        setColor(distinctColors[j % 14]);
        image.beginPath();
        image.moveTo(centers[pair[0]].x, centers[pair[0]].y);
        image.lineTo(centers[pair[1]].x, centers[pair[1]].y);
        image.stroke();
        image.closePath();
      }
      if (centers[leftEyeId]) {
        image.beginPath();
        image.arc(centers[leftEyeId].x, centers[leftEyeId].y, 10, 0, 2 * Math.PI);
        image.stroke();
      }
      if (centers[rightEyeId]) {
        image.beginPath();
        image.arc(
          centers[rightEyeId].x,
          centers[rightEyeId].y,
          10,
          0,
          2 * Math.PI
        );
        image.stroke();
      }
    }
  }


  /**
   * Subscribe to a given topic and hook the callback.
   * @param {Topic} topic - The topic.
   * @param {array} historyArray - The history array to use in the callback.
   */
  function subscribeAndBind(topic, historyArray) {
    topic.subscribe(handleTopicData.bind(this, historyArray));
  }

  /**
   * Add upcomming data from subscribed topic into the history buffer.
   * @param {Topic} historyTopic - The history topic.
   * @param {object} msg - The new data.
   */
  function handleTopicData(historyTopic, msg) {
    historyTopic.push(msg);
    // 50 kinect images ~= 2.6 mb
    if (historyTopic.length >= 50) {
      historyTopic.shift();
    }
    draw();
  }
}
