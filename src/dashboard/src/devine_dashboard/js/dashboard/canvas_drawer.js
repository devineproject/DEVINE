import { RosTopic } from "./ros";
import { distinctColors } from "../vars/colors";
import $ from "jquery";

const BODY_PARTS = {
  EYES: {
    LEFT: 15,
    RIGHT: 14
  },
  AGGR: [
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
  ]
};

export default class CanvasDrawer {
  /**
   * Class who draws over a selected canvas element.
   * @param {dict} devineTopics - The list of ros topics.
   * @param {string} canvasId - The html element canvas id.
   * @param {func|undefined} updatePositionsCallback - Object found callback.
   * @param {func|undefined} updateDelayLengthCallback - Current delay lenght callback.
   */
  constructor(devineTopics, canvasId, updatePositionsCallback=undefined, updateDelayLengthCallback=undefined) {
    this.updatePositionsCallback = updatePositionsCallback;
    this.updateDelayLengthCallback = updateDelayLengthCallback;

    this.topics = {
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

    this.canvas = this._getCanvas(canvasId);
    this.canvasSize = { x: 640, y: 480 };
    this._drawNoFeed();
    
    this.history = this._initHistoryBuffers();
    this.changeImageSource('image_raw');

    this._initConfidenceReset();
  }

  /** Draw current image on canvas */
  draw() {
    // TODO: const draw = throttle(function draw() {}, 100);
    let currentImage = this._getCurrentHistoryElement(this.history.image);
    let stamp = currentImage ? currentImage.header.stamp : null;
    let obj_pos_2d = this._getCurrentHistoryElement(this.history.buffers.object_position_2d, stamp);
    let obj_pos_3d = this._getCurrentHistoryElement(this.history.buffers.object_position_3d, stamp);
    let body_tracking = this._getCurrentHistoryElement(this.history.buffers.body_tracking);
    let confidence = this.history.position == 1 ? this._getCurrentHistoryElement(this.history.buffers.confidence) : undefined;
    let segmentation = this._getCurrentHistoryElement(this.history.buffers.segmentation, stamp);
    
    if (this.updatePositionsCallback) {
      this.updatePositionsCallback(obj_pos_2d, obj_pos_3d);
    }

    if (currentImage != undefined) {
      if (this.updateDelayLengthCallback) {
        this.updateDelayLengthCallback(this.history.image.length);
      }

      let imageObject = new Image();
      imageObject.onload = () => {
        this._resetCanvas(imageObject);

        if (obj_pos_2d != undefined) {
          this._drawPositionFound(obj_pos_2d.point.x, this.canvasSize.y - obj_pos_2d.point.y);
        }

        if (segmentation != undefined) {
          this._drawObjectsRectangles(segmentation.objects, confidence ? confidence.data : undefined);
        }

        if (body_tracking != undefined) {
          this._drawBodyTracking(JSON.parse(body_tracking.data));
        }
      };
      imageObject.src = "data:image/" + currentImage.format + ";base64, " + currentImage.data;
    }
  }

  /**
   * Change the history position.
   * @param {number} position - The new position.
   */
  setHistoryPosition(position) {
    this.history.position = position;
    this.draw();
  }

  /**
   * Switch camera image source.
   * @param {string} newSource - The image source to use.
   */
  changeImageSource(newSource) {
    switch (newSource) {
      case 'segmentation_image':
      case 'body_tracking_image':
      case 'zone_detection_image':
      case 'image_raw':
        this.history.image = this.history.buffers[newSource];
        break;
      default:
        this.history.image = this.history.buffers['image_raw'];
        break;
    }
  }
  
  /**
   * Retreive the selected canvas html element.
   * @param {number} id - The canvas id.
   * @return {object} The element.
   */
  _getCanvas(id) {
    const canvas = $(id)[0];
    if (!canvas) {
      throw "No canvas element detected.";
    } else {
      return canvas.getContext("2d");
    }
  }

  /** 
   * Subscribe all buffers to every topics
   * @return {object} The initialized history.
   */
  _initHistoryBuffers() {
    let buffers = {};
    Object.keys(this.topics).forEach(topic => {
      if (buffers[topic] === undefined) {
        buffers[topic] = [];
      }
      this.topics[topic].subscribe(this._handleTopicData.bind(this, buffers[topic]));
    });

    return {
      position: 1,
      image: [],
      buffers: buffers
    };
  }

  /** Special case, reset confidence each game */
  _initConfidenceReset() {
    this.topics.segmentation.subscribe(function(confidence, _) {
      confidence.length = 0;
    }.bind(this, this.history.buffers.confidence));
  }

  /**
   * Add upcomming data from subscribed topic into the history buffer.
   * @param {Topic} historyTopic - The history topic.
   * @param {object} msg - The new data.
   */
  _handleTopicData(historyTopic, msg) {
    historyTopic.push(msg);
    // 50 kinect images ~= 2.6 mb
    if (historyTopic.length >= 50) {
      historyTopic.shift();
    }
    this.draw();
  }

  /**
   * Set the image canvas color brush for further drawing.
   * @param {string} color - The new color.
   */
  _setColor(color) {
    if (color) {
      this.canvas.strokeStyle = color;
      this.canvas.fillStyle = color;
    }
  }

  /** Draw an old-school style ne camera feed warning */
  _drawNoFeed() {
    this._setColor("red");
    this.canvas.font = "bold 20pt Arial";
    this.canvas.fillText("< No Camera Feed />", 190, this.canvasSize.y / 2);
  }

  /**
   * Draw a cross an the position found
   * @param {int} x - X coordinate.
   * @param {int} y - Y coordinate.
   */
  _drawPositionFound(x, y) {
    this._setColor("#3498DB");
    this.canvas.fillText("Found!", x + 8, y - 8);
    this.canvas.fillRect(x - 3, y - 3, 6, 6);
    this.canvas.fillRect(x - 20, y - 1, 40, 2);
    this.canvas.fillRect(x - 1, y - 20, 2, 40);
  }

  /**
   * Draw rectangle over object found on the image.
   * @param {array} objects - The list of objects.
   * @param {int|undefined} confidence - The confidence value, if an object was found.
   */
  _drawObjectsRectangles(objects, confidence) {
    for (var i = 0; i < objects.length; i++) {
      this._setColor(distinctColors[i % 14]);

      this.canvas.beginPath();
      let {x_offset, y_offset, height, width} = objects[i].bounding_box;

      this.canvas.rect(x_offset, y_offset, width, height);

      this.canvas.fillText(objects[i].category_name, x_offset, y_offset - 1);

      // Show the confidence about the object found, if any.
      if (confidence !== undefined && confidence.length === objects.length) {
        let obj_name_offset = this.canvas.measureText(objects[i].category_name).width;
        this.canvas.fillText(`(${(confidence[i]*100).toFixed(2)}%)`, x_offset + obj_name_offset + 5, y_offset - 1);
      }

      this.canvas.stroke();
      this.canvas.closePath();
    }
  }
  
  /**
   * Clear the canvas and reset the image.
   * @param {object} imageObject - The image data.
   */
  _resetCanvas(imageObject) {
    this.canvas.clearRect(0, 0, this.canvasSize.x, this.canvasSize.y);
    this.canvas.drawImage(imageObject, 0, 0);
    this.canvas.font = "bold 12pt Arial";
    this.canvas.lineWidth = "2";
  }
  
  /**
   * Retreive the current history element with the current position.
   * @param {array} historyArray - The array to take the element from.
   * @return {object} The element.
   */
  _getCurrentHistoryElement(historyArray, stamp=null) {
    if (!stamp) {
      // Return latest element if there is no stamp
      return historyArray[historyArray.length - this.history.position];
    }
    for (let i = historyArray.length - 1; i >= 0; --i) {
      if (Math.abs(historyArray[i].header.stamp.secs - stamp.secs) < 2) {
        return historyArray[i];
      }
    }
  }

  /**
   * Map body part with one another (e.g.: ear with eye).
   * Adaptation and refactor of function draw_humans in tf_pose/estimator.py
   * @param {array} humans - The list of humans detected.
   *
   * @see CocoPairs in tf_pose/common.py
   */
  _drawBodyTracking(humans) {
    for (let i in humans) {
      let centers = {};

      // For each body parts, find its pixel location in the image
      for (let j in humans[i].body_parts) {
        let bp = humans[i].body_parts[j];
        centers[bp.index] = {
          x: bp.x * this.canvasSize.x + 0.5,
          y: bp.y * this.canvasSize.y + 0.5
        };
      }

      // For each body parts that should be linked, draw a line between these two
      for (let j in BODY_PARTS.AGGR) {
        let pair = BODY_PARTS.AGGR[j];
        if (!centers[pair[0]] || !centers[pair[1]]) {
          continue;
        }
        this._setColor(distinctColors[j % 14]);
        this.canvas.beginPath();
        this.canvas.moveTo(centers[pair[0]].x, centers[pair[0]].y);
        this.canvas.lineTo(centers[pair[1]].x, centers[pair[1]].y);
        this.canvas.stroke();
        this.canvas.closePath();
      }

      const drawEye = (eyeId) => {
        if (centers[eyeId]) {
          this.canvas.beginPath();
          this.canvas.arc(centers[eyeId].x, centers[eyeId].y, 10, 0, 2 * Math.PI);
          this.canvas.stroke();
        }
      };

      drawEye(BODY_PARTS.EYES.LEFT);
      drawEye(BODY_PARTS.EYES.RIGHT);
    }
  }
}
