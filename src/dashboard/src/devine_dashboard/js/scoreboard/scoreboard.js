import $ from 'jquery';
import ROSLIB from 'roslib';

class Player {
  constructor(playerNameOrCopiedObj, inGamePlayed = 0, inGameWon = 0) {
    if (typeof playerNameOrCopiedObj === "string") {
      // player ctor
      this.name = playerNameOrCopiedObj;
      this.gamePlayed = inGamePlayed;
      this.gameWon = inGameWon;
    } else {
      // Copy ctor
      playerNameOrCopiedObj && Object.assign(this, playerNameOrCopiedObj);
    }
  }
  get percentageWon() {
    if (this.gamePlayed === 0) {
      return "0.00";
    }
    return parseFloat(Math.round(this.gameWon * 10000 / this.gamePlayed) / 100).toFixed(2);
  }
}

class BrowserStorage {
  set leaderboard(leaderboardArray) {
    //Enforce array storage to not corrupt localstorage
    if (typeof leaderboardArray !== "object" || !leaderboardArray.sort) {
      throw "LeaderboardArray is not an array";
    }
    localStorage.devineLeaderboard = JSON.stringify(leaderboardArray);
  }
  get leaderboard() {
    if (localStorage.devineLeaderboard === undefined) {
      return [];
    }
    let leaderboard = JSON.parse(localStorage.devineLeaderboard);
    return leaderboard.map(object => new Player(object));
  }
}

export default function createScoreboard(rosTopics) {
  let browserStorage = new BrowserStorage();
  let leaderboard = browserStorage.leaderboard;
  let playerName = "Anonymous"; //Unregistered user
  fillLeaderboardView(leaderboard);

  const rosUrl = `ws://${window.location.hostname}:9090`;
  const ros = new ROSLIB.Ros({ url: rosUrl });

  //TODO: republish to scoreboardTopic, sync up with potential other instances open
  let [nameTopic, gameSuccessTopic, scoreboardTopic] = 
    [rosTopics.player_name, rosTopics.object_guess_success, rosTopics.scoreboard]
      .map(top => new ROSLIB.Topic({ros, name: top.name, messageType: top.type}));

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

    browserStorage.leaderboard = leaderboard; //update browser storage
    fillLeaderboardView(leaderboard); //update view
  });
}



function fillLeaderboardView(leaderboardArray) {  
  let leaderboardHtml = leaderboardArray
    //.map(object => object instanceof Player ? object : new Player(object)) //Map as Player class -- Shouldn't be necessary
    .sort((player1, player2) => player2.percentageWon - player1.percentageWon) //Order by percentageWon
    .map((player, index) => { //Remap to html to be injected
      return `<tr>
        <th scope="row">${index+1}</th>
        <td>${player.name}</td>
        <td>${player.gamePlayed}</td>
        <td>${player.gameWon}</td>
        <td>${player.percentageWon} %</td>
      </tr>`;
    }).join("");

  $('#tbody-leaderboard').html(leaderboardHtml);
}
