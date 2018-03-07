import ros from './ros';
import ROSLIB from 'roslib';
import $ from 'cash-dom';

const listener = new ROSLIB.Topic({
    ros : ros,
    name : '/listener',
    messageType : 'std_msgs/String'
});

$('input[type="checkbox"').on("change", function() {
  const view = $('.command-view[name="test"]').find('.subscriber-log')[0];
  if(this.checked) {
    listener.subscribe(function(message) {
        view.innerHTML += `${message.data}\n`
        view.scrollTop = view.scrollHeight;
    });
  } else {
    listener.unsubscribe();
    view.innerHTML += '=======Unsubscribed=======\n';
    view.scrollTop = view.scrollHeight;
  }
});


