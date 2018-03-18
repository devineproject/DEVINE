import ros from '../ros';
import ROSLIB from 'roslib';
import $ from 'cash-dom';

const listener = new ROSLIB.Topic({
  ros: ros,
  name: '/game_system_state',
  messageType: 'std_msgs/String'
});

$('.command-view[name="gamestate"]').find('input[type="checkbox"]').on("change", function () {
  const view = $('.command-view[name="gamestate"]').find('.subscriber-log')[0];
  if (this.checked) {
    listener.subscribe(function (message) {
      view.innerText += `${message.data}\n`;
      view.scrollTop = view.scrollHeight;
    });
  } else {
    listener.unsubscribe();
    view.innerText += '=======Unsubscribed=======\n';
    view.scrollTop = view.scrollHeight;
  }
});


