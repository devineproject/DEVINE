import ros from '../ros';
import LogConsole from '../console'
import ROSLIB from 'roslib';
import $ from 'cash-dom';

const cons = new LogConsole("Kinect", "#3498DB");
const cameraCheckbox = $("#camera_checkbox");
const segmentationCheckbox = $("#segmentation_checkbox");
const objectPos2d = $("#kinect_pos_found");
const objectPos3d = $("#kinect_pos_calc");
const canvas = $("#kinect_image")[0];
const image = canvas.getContext("2d");
const history = $('#kinect_image_history');

const topicListeners = {
  image: new ROSLIB.Topic({
    ros: ros,
    name: '/camera/rgb/image_color/compressed',
    messageType: 'sensor_msgs/CompressedImage'
  }),
  segmentation_image: new ROSLIB.Topic({
    ros: ros,
    name: '/devine/image',
    messageType: 'sensor_msgs/CompressedImage'
  }),
  segmentation: new ROSLIB.Topic({
    ros: ros,
    name: "/rcnn_segmentation",
    messageType: 'std_msgs/String'
  }),
  object_position_2d: new ROSLIB.Topic({
    ros: ros,
    name: '/object_found',
    messageType: 'std_msgs/Int32MultiArray'
  }),
  object_position_3d: new ROSLIB.Topic({
    ros: ros,
    name: '/object_location',
    messageType: 'std_msgs/Float32MultiArray'
  })
};

const topicHistory = {
  image: [],
  segmentation_image: [],
  segmentation: [],
  object_position_2d: [],
  object_position_3d: [],
  position: 1
};

cameraCheckbox.on("change", function() {
  if (this.checked) {
    topicListeners.image.subscribe(handleTopicData.bind(this, 'image'));
    topicListeners.object_position_2d.subscribe(handleTopicData.bind(this, 'object_position_2d'));
    topicListeners.object_position_3d.subscribe(handleTopicData.bind(this, 'object_position_3d'));
    if (segmentationCheckbox.is(":checked")) {
      topicListeners.segmentation.subscribe(handleTopicData.bind(this, 'segmentation'));
      topicListeners.segmentation_image.subscribe(handleTopicData.bind(this, 'segmentation_image'));
    }
    cons.log("Camera subscribed");
  } else {
    for (let i in topicListeners) {
      topicListeners[i].unsubscribe();
    }
    cons.log("Camera unsubscribed");
    setTimeout(() => drawNoFeed(), 200);
  }
});

segmentationCheckbox.on("change", function() {
  if (this.checked && cameraCheckbox.is(":checked")) {
    topicListeners.segmentation.subscribe(handleTopicData.bind(this, 'segmentation'));
    topicListeners.segmentation_image.subscribe(handleTopicData.bind(this, 'segmentation_image'));
    cons.log("Segmentation subscribed");
  } else {
    topicListeners.segmentation.unsubscribe();
    topicListeners.segmentation_image.unsubscribe();
    cons.log("Segmentation unsubscribed");
  }
});

$('#kinect_image_type').on("change", function () {
  topicListeners.image.name = this.value;
});

history.on("change", function() {
  topicHistory.position = Math.max(this.value, 1);
  draw();
});

function drawNoFeed() {
  image.fillStyle = "red";
  image.font = "bold 20pt Arial";
  image.fillText("< No Camera Feed />", 190, (canvas.height / 2));
}

drawNoFeed(); // No feed at startup

//We want to limit drawing for performance, yet we might want to keep all data
const draw = throttle(function draw() {
  let obj_pos_2d = getElem(topicHistory.object_position_2d);
  let obj_pos_3d = getElem(topicHistory.object_position_3d);
  let image_length = 0, seg, img;

  if (segmentationCheckbox.is(":checked")) {
    image_length = topicHistory.segmentation_image.length;
    seg = getSeg();
    img = getElem(topicHistory.segmentation_image);
  } else {
    image_length = topicHistory.image.length;
    img = getElem(topicHistory.image);
  } 

  if (obj_pos_2d != undefined) {
    objectPos2d.innerText = `(${obj_pos_2d[0]}, ${obj_pos_2d[1]})`;
  }

  if (obj_pos_3d != undefined) {
    let cleanFloat = number => number===null ? "N/A" : number.toFixed(2);
    objectPos3d.innerText = `(${cleanFloat(obj_pos_3d[0])}, ` +
      `${cleanFloat(obj_pos_3d[1])}, ${cleanFloat(obj_pos_3d[2])})`;
  }

  if (img != undefined) {
    history.prop('max', image_length);
    let imageObject = new Image();
    imageObject.onload = function() {
      image.clearRect(0, 0, 640, 480);
      image.beginPath();
      image.drawImage(imageObject, 0, 0);
      if (seg != undefined) {
        let segmentedDataObj = JSON.parse(seg);
        let objs = segmentedDataObj.objects;
        if (objs) {
          objs.forEach(obj => {
            let [left, top, height, width] = obj.bbox;
            image.rect(left, 480-top-height, width, height);
            image.fillText(obj.category, left, 480-top-height-1);
          });
        }
      }
      image.stroke();
    };
    imageObject.src = "data:image/jpg;base64, " + img;  
  }  
}, 100);

function handleTopicData(topic, msg) {
  topicHistory[topic].push(msg.data);
  // 50 kinect images ~= 2.6 mb
  if (topicHistory[topic].length >= 50) {
    topicHistory[topic].shift();
  }
  draw();
}

function getElem(arr) {
  return arr[arr.length - topicHistory.position];
}

function getSeg() {
  return topicHistory.segmentation[topicHistory.segmentation_image.length - topicHistory.position];
}

// Taken from underscore.js
// Returns a function, that, when invoked, will only be triggered at most once
// during a given window of time. Normally, the throttled function will run
// as much as it can, without ever going more than once per `wait` duration;
// but if you'd like to disable the execution on the leading edge, pass
// `{leading: false}`. To disable execution on the trailing edge, ditto.
function throttle(func, wait, options) {
  var context, args, result;
  var timeout = null;
  var previous = 0;
  if (!options) options = {};
  var later = function () {
    previous = options.leading === false ? 0 : Date.now();
    timeout = null;
    result = func.apply(context, args);
    if (!timeout) context = args = null;
  };
  return function () {
    var now = Date.now();
    if (!previous && options.leading === false) previous = now;
    var remaining = wait - (now - previous);
    context = this;
    args = arguments;
    if (remaining <= 0 || remaining > wait) {
      if (timeout) {
        clearTimeout(timeout);
        timeout = null;
      }
      previous = now;
      result = func.apply(context, args);
      if (!timeout) context = args = null;
    } else if (!timeout && options.trailing !== false) {
      timeout = setTimeout(later, remaining);
    }
    return result;
  };
}
