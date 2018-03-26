import ros from '../ros';
import ROSLIB from 'roslib';
import $ from 'cash-dom';

var listener = new ROSLIB.Topic({
  ros: ros,
  name: '/camera/rgb/image_color/compressed',
  messageType: 'sensor_msgs/CompressedImage'
});

var posListener = new ROSLIB.Topic({
  ros: ros,
  name: '/object_found',
  messageType: 'std_msgs/Int32MultiArray'
});

var calcListener = new ROSLIB.Topic({
  ros: ros,
  name: '/object_location',
  messageType: 'std_msgs/Float32MultiArray'
});

$('.command-view[name="kinect"]').find('input[type="checkbox"]').on("change", function () {
  toggle_video(this.checked);
});

$('#kinect_image_type').on("change", function () {
  toggle_video(false);
  listener.name = this.value;
  toggle_video($('.command-view[name="kinect"]')
    .find('input[type="checkbox"]')
    .is(":checked"));
});

function toggle_video(play) {
  if (play) {
    var pos_found = null;
    const found_ctx = document.getElementById("kinect_pos_found");
    const found_pos_ctx = document.getElementById("kinect_image_obj_position");
    const calc_ctx = document.getElementById("kinect_pos_calc");
    const ctx = document.getElementById("kinect_image");
    listener.subscribe(throttle(function (message) {
      ctx.src = "data:image/jpg;base64, " + message.data;
      if (pos_found !== null) {
        found_pos_ctx.style.visibility = "visible";
        found_pos_ctx.style.left = `${pos_found[0]}px`;
        found_pos_ctx.style.top = `${pos_found[1]}px`;
      }
    }, 100));

    posListener.subscribe(throttle(function (message) {
      if (message && message.data) {
        console.log(`Position Found: (${message.data[0]}, ${message.data[1]})`);
        found_ctx.innerText = `(${message.data[0]}, ${message.data[1]})`;
        pos_found = message.data;
      }
    }, 1000));

    var cleanFloat = number => number===null ? "N/A" : number.toFixed(2);

    calcListener.subscribe(throttle(function (message) {
      if (message && message.data) {
        console.log(`Position Calculated: (${message.data[0]}, ${message.data[1]})`);
        calc_ctx.innerText = `(${cleanFloat(message.data[0])}, `+
        `${cleanFloat(message.data[1])}, ${cleanFloat(message.data[2])})`;
      }
    }, 1000));
  } else {
    listener.unsubscribe();
    posListener.unsubscribe();
    calcListener.unsubscribe();
  }
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