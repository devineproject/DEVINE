import $ from "jquery";
import dashboard from "./dashboard/dashboard";
import demo from "./demo/demo";
import scoreboard from "./scoreboard/scoreboard";

try {
  // Fix popper.js for Bootstrap4
  window.$ = window.jQuery = require("jquery");
  window.Popper = require("popper.js").default;
  require("bootstrap");
} catch (e) {
  console.error(e);
}

$.getJSON("/topics", function(topics) {
  switch (window.route) {
    case "scoreboard":
      scoreboard(topics);
      break;
    case "debug":
      dashboard(topics);
      break;
    case "index": //Passthrough as default
    case "demo":
    default:
      demo(topics);
      break;
  }
});
