import ROSLIB from 'roslib';
import { RosTopic, ros } from './ros';
import { devineTopics } from './vars/devine_topics'
import LogConsole from './console'
import $ from 'jquery';

const cons = new LogConsole("ROS", "grey")

/** Send message to selected topic */
function publish() {
  var topic = $("#publisher_topic").val() || $("#publisher_topic").attr("placeholder");
  var message = $("#publisher_message").val() || $("#publisher_message").attr("placeholder");

  new RosTopic({
    name: topic,
    type: 'std_msgs/String'
  }).publish({ data: message });

  if ($("#show_pub_checkbox").is(':checked')) {
    cons.log(`[${topic}] ${message}`);
  }
}

$("#publisher_publish").on("click", publish);
$('#publisher_message').keypress(function (key) {
  if (key.which == 13) {
    publish();
  }
});

/** Write selected topic to textbox */
function setTopicClick() {
  $("#publisher_topics .dropdown-item").on("click", function () {
    $("#publisher_topic").val(this.text);
  });
}

/** Insert a list of topics after an element */
function insertTopics(topicsList, afterElement) {
  topicsList.forEach(topic =>
    $(`<a class="dropdown-item"></a>`).text(topic).insertAfter(afterElement));
  setTopicClick();
}

/** Generate list of available topics */
function setTopicsList(rosTopics) {
  if (rosTopics.length > 0) {
    insertTopics(rosTopics.topics, ".all-topics");
  } else {
    insertTopics(["No topic available"], ".all-topics");
  }
}

insertTopics(Object.keys(devineTopics).map((key) => devineTopics[key].name), ".common-topics");
ros.getTopics(setTopicsList);

