import $ from "jquery";

export default class LogConsole {
  constructor(source, color) {
    this.console = $("#console");
    this.source = source;
    this.color = color;

    $("#clear_console").on("click", () => this.clear());
  }

  log(message) {
    var logMessage = $("<span><b><font></font></b><span></span><br/></span>");
    $("font", logMessage)
      .text(this.source)
      .prop("color", this.color);
    $("span", logMessage).text(` > ${message}`);

    this.console.prepend(logMessage);
    this.console.scrollTop = this.console.scrollHeight;
  }

  clear() {
    this.console.html("");
  }
}
