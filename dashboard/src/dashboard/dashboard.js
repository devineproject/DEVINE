import publisher from './publisher';
import gamestate from './subscribers/gamestate';
import kinect from './subscribers/kinect';
import snips from './subscribers/snips';
import guesswhat from './subscribers/guesswhat';

export default function CreateDashboard() {
  $(document).ready(function () {
    // Uncheck all subscriptions
    $('[type="checkbox"]').prop('checked', false);
  });

  kinect();
  // TODO: IMPLEMENT OTHER MODULE CONSTRUCTORS
}