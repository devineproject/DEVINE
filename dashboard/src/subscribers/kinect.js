import ros from '../ros';
import ROSLIB from 'roslib';
import $ from 'cash-dom';

var listener = new ROSLIB.Topic({
  ros: ros,
  name: '/camera/rgb/image_color/compressed',
  messageType: 'sensor_msgs/CompressedImage'
});

$('.command-view[name="kinect"]').find('input[type="checkbox"]').on("change", function () {
  toggle_video(this.checked);
});

$('#kinect_image_type').on("change", function() {
  toggle_video(false);
  listener.name = this.value;
  toggle_video($('.command-view[name="kinect"]')
    .find('input[type="checkbox"]')
    .is(":checked"));
});

function toggle_video(play) {
  if (play) {
    const ctx = document.getElementById("kinect_image");
    listener.subscribe(function (message) {
      ctx.src = "data:image/jpg;base64, " + message.data;
    });
  } else {
    listener.unsubscribe();
  }
}