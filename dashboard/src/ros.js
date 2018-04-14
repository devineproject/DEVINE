import ROSLIB from 'roslib';
import LogConsole from './console'
import $ from 'jquery';

const cons = new LogConsole("ROS", "grey")

const ros = new ROSLIB.Ros({
  url: `ws://${window.location.hostname}:9090`
});

ros.on('connection', () => {
  cons.log('Rosbridge connection established');
  ros.getTopics(setTopicsList);
});
ros.on('error', () => cons.log('Rosbridge connection error'));
ros.on('close', () => cons.log('Rosbridge connection closed'));

function setTopicsList(topics) {
  for (var i in topics.topics) {
    $("#publisher_topics").append(`<a class="dropdown-item">${topics.topics[i]}</a>`);
  }
}

$("#publisher_topics .dropdown-item").on("click", function() {
  $("#publisher_topic").val(this.text);
});

$("#publisher_publish").on("click", function() {
  var topic = $("#publisher_topic").val() || $("#publisher_topic").attr("placeholder");
  var message = $("#publisher_message").val() || $("#publisher_message").attr("placeholder");

  const cmdTopic = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: 'std_msgs/String'
  });
  cmdTopic.publish({data:message});
  cons.log(`[${topic}] ${message}`);
});

export default ros;
