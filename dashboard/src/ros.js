import ROSLIB from 'roslib';
import LogConsole from './console'

const cons = new LogConsole("ROS", "grey")

const ros = new ROSLIB.Ros({
  url: `ws://${window.location.hostname}:9090`
});

ros.on('connection', () => cons.log('Rosbridge connection established'))
ros.on('error', () => cons.log('Rosbridge connection error'))
ros.on('close', () => cons.log('Rosbridge connection closed'))

export default ros;
