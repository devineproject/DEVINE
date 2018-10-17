import $ from 'jquery';
import publisher from './publisher';
import gamestate from './subscribers/gamestate';
import kinect from './subscribers/kinect';
import dialog from './subscribers/dialog';
import guesswhat from './subscribers/guesswhat';
import imgdispatcher from './subscribers/image_dispatcher';

export default function CreateDashboard(topics) {
  $(document).ready(function () {
    // Uncheck all subscriptions
    $('[type="checkbox"]').prop('checked', false);

    publisher(topics);
    kinect(topics);
    dialog(topics);
    gamestate(topics);
    guesswhat(topics);
    imgdispatcher(topics);
  });
}
