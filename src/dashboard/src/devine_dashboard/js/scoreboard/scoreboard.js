import $ from "jquery";
import ROSLIB from "roslib";

/** Class representing a player on the leaderboard. */
class Player {
  constructor(playerNameOrCopiedObj, gamePlayed = 0, gameWon = 0) {
    if (typeof playerNameOrCopiedObj === "string") {
      // player ctor
      this.name = playerNameOrCopiedObj;
      this.gamePlayed = gamePlayed;
      this.gameWon = gameWon;
    } else {
      // Copy ctor
      playerNameOrCopiedObj && Object.assign(this, playerNameOrCopiedObj);
    }
  }

  /** Get the percentage won. */
  get percentageWon() {
    if (this.gamePlayed === 0) {
      return "0.00";
    }
    return parseFloat(
      Math.round((this.gameWon * 10000) / this.gamePlayed) / 100
    ).toFixed(2);
  }
}

/** Class allowing to save the leaderboard in the localstorage. */
class BrowserStorage {
  /**
   * Save the leaderboard.
   * @param {array} leaderboardArray - The list of player in the leaderboard.
   */
  set leaderboard(leaderboardArray) {
    //Enforce array storage to not corrupt localstorage
    if (typeof leaderboardArray !== "object" || !leaderboardArray.sort) {
      throw "LeaderboardArray is not an array";
    }
    localStorage.devineLeaderboard = JSON.stringify(leaderboardArray);
  }

  /** Get the leaderboard. */
  get leaderboard() {
    if (localStorage.devineLeaderboard === undefined) {
      return [];
    }
    let leaderboard = JSON.parse(localStorage.devineLeaderboard);
    return leaderboard.map(object => new Player(object));
  }
}

/**
 * Update the scoreboard when a player complete a game.
 * @param {dict} rosTopics - The available topics.
 */
export default function createScoreboard(rosTopics) {
  let browserStorage = new BrowserStorage();
  let leaderboard = browserStorage.leaderboard;
  let playerName = "Anonymous"; //Unregistered user
  fillLeaderboardView(leaderboard);

  const rosUrl = `ws://${window.location.hostname}:9090`;
  const ros = new ROSLIB.Ros({ url: rosUrl });

  //TODO: republish to scoreboardTopic, sync up with potential other instances open
  let [nameTopic, gameSuccessTopic, scoreboardTopic] = [
    rosTopics.player_name,
    rosTopics.object_guess_success,
    rosTopics.scoreboard
  ].map(
    top => new ROSLIB.Topic({ ros, name: top.name, messageType: top.type })
  );

  nameTopic.subscribe(newPlayerName => {
    playerName = newPlayerName.data;
  });

  gameSuccessTopic.subscribe(game_won => {
    let player;
    for (let i in leaderboard) {
      if (leaderboard[i].name === playerName) {
        player = leaderboard[i];
        break;
      }
    }

    if (player === undefined) {
      player = new Player(playerName);
      leaderboard.push(player);
    }

    ++player.gamePlayed;
    if (game_won.data === true) {
      ++player.gameWon;
    }

    browserStorage.leaderboard = leaderboard;
    fillLeaderboardView(leaderboard);
  });
}

/**
 * Convert a list of player stats to an html leaderboard and update the view.
 * @param {array} leaderboardArray - The list of player in the leaderboard.
 */
function fillLeaderboardView(leaderboardArray) {
  let leaderboardHtml = leaderboardArray
    //.map(object => object instanceof Player ? object : new Player(object)) //Map as Player class -- Shouldn't be necessary
    .sort((player1, player2) => player2.percentageWon - player1.percentageWon) //Order by percentageWon
    .map((player, index) => {
      //Remap to html to be injected
      return `<tr>
        <th scope="row">${index + 1}</th>
        <td>${player.name}</td>
        <td>${player.gamePlayed}</td>
        <td>${player.gameWon}</td>
        <td>${player.percentageWon} %</td>
      </tr>`;
    })
    .join("");

  $("#tbody-leaderboard").html(leaderboardHtml);
}
