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
  object_position_3d: new RosTopic(devineTopics.object_location)
};

const history = {
  image: [],
  segmentation_image: [],
  segmentation: [],
  object_position_2d: [],
  object_position_3d: [],
  position: 1
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
    let [left, top, width, height] = objects[i].bbox;
    image.rect(left, top, width, height);
    image.fillText(objects[i].category, left, top - 1);
    image.stroke();
    image.closePath();
  }
}

function getCurrentElement(array) {
  return array[array.length - history.position];
}

function getSegmentation() {
  return topicHistory.segmentation[topicHistory.segmentation_image.length - topicHistory.position];
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
    };
    imageObject.src = "data:image/jpg;base64, " + img;
  }
}, 100);

drawNoFeed(); // No feed at startup