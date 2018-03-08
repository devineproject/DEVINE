import ROSLIB from 'roslib';

const ros = new ROSLIB.Ros({
  url: `ws://${window.location.hostname}:9090`
});

ros.on('connection', () => console.log('Rosbridge connection established'))
ros.on('error', () => console.log('Rosbridge connection error'))
ros.on('close', () => console.log('Rosbridge connection closed'))

export default ros;
