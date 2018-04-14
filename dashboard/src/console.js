import $ from 'jquery'

export default class LogConsole {
    constructor(source, color) {
        this.console = document.getElementById("console");
        this.source = source;
        this.color = color;
    }

    log(message) {
        this.console.innerHTML += `<b><font color="${this.color}">${this.source}</font></b> > ${message}<br/>`;
        this.console.scrollTop = this.console.scrollHeight;
    }
}