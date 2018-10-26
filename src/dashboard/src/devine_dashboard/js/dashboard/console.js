import $ from "jquery";

/** Class that handle visual system logging in the dashboard view */
export default class LogConsole {
  constructor(source, color) {
    this.console = $("#console");
    this.source = source;
    this.color = color;

    $("#clear_console").on("click", () => this.clear());
  }

  /**
   * Log a message.
   * @param {string} message - The message.
   */
  log(message) {
    var logMessage = $("<span><b><font></font></b><span></span><br/></span>");
    $("font", logMessage)
      .text(this.source)
      .prop("color", this.color);
    $("span", logMessage).text(` > ${message}`);

    this.console.prepend(logMessage);
    this.console.scrollTop = this.console.scrollHeight;
  }

  /** Clear the logs. */
  clear() {
    this.console.html("");
  }
}
