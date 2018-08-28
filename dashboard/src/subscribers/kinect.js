import { RosTopic } from '../ros';
import { distinctColors } from '../vars/colors'
import devineTopics from '../vars/devine_topics.json'
import throttle from '../throttle'
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'jquery';

const cons = new LogConsole("Kinect", "#3498DB");
const cameraSubscriber = $("#camera_checkbox");
const segmentationSubscriber = $("#segmentation_checkbox");
const bodyTrackingSubscriber = $("#body_tracking_checkbox");

const objectPos2d = $("#kinect_pos_found")[0];
const objectPos3d = $("#kinect_pos_calc")[0];
const image = $("#kinect_image")[0].getContext("2d");
const imageSize = { x: 640, y: 480 }
const delay = $('#kinect_image_delay');

const topics = {
  image:              new RosTopic(devineTopics.image),
  segmentation_image: new RosTopic(devineTopics.segmentation_image),
  segmentation:       new RosTopic(devineTopics.segmentation),
  object_position_2d: new RosTopic(devineTopics.object_found),
  object_position_3d: new RosTopic(devineTopics.object_location),
  body_position:      new RosTopic(devineTopics.body_position)
};

const history = {
  image: [],
  segmentation_image: [],
  segmentation: [],
  object_position_2d: [],
  object_position_3d: [],
  position: 1,
  body_tracking: []
};

function handleTopicData(historyTopic, msg) {
  historyTopic.push(msg.data);
  // 50 kinect images ~= 2.6 mb
  if (historyTopic.length >= 50) {
    historyTopic.shift();
  }
  draw();
}

cameraSubscriber.on("change", function () {
  if (this.checked) {
    topics.image.subscribe(handleTopicData.bind(this, history.image));
    topics.object_position_2d.subscribe(handleTopicData.bind(this, history.object_position_2d));
    topics.object_position_3d.subscribe(handleTopicData.bind(this, history.object_position_3d));
    if (segmentationSubscriber.is(':checked')) {
      topics.segmentation.subscribe(handleTopicData.bind(this, history.segmentation));
      topics.segmentation_image.subscribe(handleTopicData.bind(this, history.segmentation_image));
    } 
    if (bodyTrackingSubscriber.is(":checked")) {
      topics.body_position.subscribe(handleTopicData.bind(this, history.body_tracking));
    }

    cons.log("Camera subscribed");
  } else {
    for (let i in topics) {
      topics[i].removeAllListeners();
    }

    cons.log("Camera unsubscribed");
    setTimeout(() => drawNoFeed(), 200);
  }
});

segmentationSubscriber.on("change", function () {
  if (this.checked && cameraSubscriber.is(':checked')) {
    topics.segmentation.subscribe(handleTopicData.bind(this, history.segmentation));
    topics.segmentation_image.subscribe(handleTopicData.bind(this, history.segmentation_image));

    cons.log("Segmentation subscribed");
  } else {
    topics.segmentation.removeAllListeners();
    topics.segmentation_image.removeAllListeners();

    cons.log("Segmentation unsubscribed");
  }
});

bodyTrackingSubscriber.on("change", function () {
  if (this.checked && cameraSubscriber.is(':checked')) {
    topics.body_position.subscribe(handleTopicData.bind(this, history.body_tracking));
    cons.log("Body tracking subscribed");
  } else {
    topics.body_position.removeAllListeners();
    cons.log("Body tracking unsubscribed");
  }
});

$('#kinect_image_type').on("change", function () {
  topics.image.name = this.value;
});

delay.on("change", function () {
  history.position = Math.max(this.value, 1);
  draw();
});

function setColor(color) {
  image.strokeStyle = color;
  image.fillStyle = color;
}

function drawNoFeed() {
  setColor("red");
  image.font = "bold 20pt Arial";
  image.fillText("< No Camera Feed />", 190, (imageSize.y / 2));
}

function drawPositionFound(x, y) {
  setColor("#3498DB");
  image.fillText("Found!", x + 8, y - 8);
  image.fillRect(x - 3, y - 3, 6, 6);
  image.fillRect(x - 20, y - 1, 40, 2);
  image.fillRect(x - 1, y - 20, 2, 40);
}

function drawObjectsRectangles(objects) {
  for (var i = 0; i < objects.length; i++) {
    setColor(distinctColors[i % 14]);

    image.beginPath();
    let [left, top, right, bottom] = objects[i].bbox;
    let width = right - left;
    let height = bottom - top;
    image.rect(left, top, width, height);
    image.fillText(objects[i].category, left, top - 1);
    image.stroke();
    image.closePath();
  }
}

function drawBodyTracking(humans) {
  //Adaptation and refactor of function draw_humans in tf_pose/estimator.py
  //Map body part with one another (e.g.: ear with eye); see CocoPairs in tf_pose/common.py
  const BodyPartsAggr = [[1, 2], [1, 5], [2, 3], [3, 4], [5, 6], [6, 7], [1, 8], [8, 9], [9, 10], [1, 11], [11, 12], [12, 13], [1, 0], [0, 14], [14, 16], [0, 15], [15, 17]];
  const leftEyeId = 15, rightEyeId = 14;
  for (let i in humans) {
    let centers = {};
    let human = humans[i];
    let partsIds = human.body_parts.map(bp => bp.index);
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
      image.arc(centers[leftEyeId].x, centers[leftEyeId].y, 10, 0, 2*Math.PI);
      image.stroke();
    }
    if (centers[rightEyeId]) {
      image.beginPath();
      image.arc(centers[rightEyeId].x, centers[rightEyeId].y, 10, 0, 2*Math.PI);
      image.stroke();
    }
  }
}

function getCurrentElement(array) {
  return array[array.length - history.position];
}

function getSegmentation() {
  return history.segmentation[history.segmentation_image.length - history.position];
}

function resetImage(image, imageObject) {
  image.clearRect(0, 0, imageSize.x, imageSize.y);
  image.drawImage(imageObject, 0, 0);
  image.font = "bold 12pt Arial";
  image.lineWidth = "2";
}

function writePositions(obj_pos_2d, obj_pos_3d) {
  if (obj_pos_2d != undefined) {
    objectPos2d.innerText = `(${obj_pos_2d[0]}, ${obj_pos_2d[1]})`;
  }

  if (obj_pos_3d != undefined) {
    let cleanFloat = number => number === null ? "N/A" : number.toFixed(2);
    objectPos3d.innerText = `(${cleanFloat(obj_pos_3d[0])}, ` +
      `${cleanFloat(obj_pos_3d[1])}, ${cleanFloat(obj_pos_3d[2])})`;
  }
}

//We want to limit drawing for performance, yet we might want to keep all data
const draw = throttle(function draw() {
  let obj_pos_2d = getCurrentElement(history.object_position_2d);
  let obj_pos_3d = getCurrentElement(history.object_position_3d);
  let body_tracking = getCurrentElement(history.body_tracking);
  let image_length = 0, seg, img;

  if (segmentationSubscriber.is(':checked')) {
    image_length = history.segmentation_image.length;
    seg = getSegmentation();
    img = getCurrentElement(history.segmentation_image);
  } else {
    image_length = history.image.length;
    img = getCurrentElement(history.image);
  }

  writePositions(obj_pos_2d, obj_pos_3d);

  if (img != undefined) {
    delay.prop('max', image_length);
    let imageObject = new Image();

    imageObject.onload = function () {
      resetImage(image, imageObject);

      if (obj_pos_2d != undefined) {
        drawPositionFound(obj_pos_2d[0], imageSize.y - obj_pos_2d[1]);
      }

      if (seg != undefined) {
        drawObjectsRectangles(JSON.parse(seg).objects);
      }

      if (body_tracking != undefined) {
        drawBodyTracking(JSON.parse(body_tracking));
      }
    };
    imageObject.src = "data:image/jpg;base64, " + img;
  }
}, 100);

drawNoFeed(); // No feed at startup