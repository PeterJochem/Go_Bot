import './App.css';
import React from 'react';
import BoardRow from "./board_row.js";

export default class Game extends React.Component {
  
  constructor(props) {
	super(props);
	this.render = this.render.bind(this);
	this.create_empty_board = this.create_empty_board.bind(this);
	this.parse_game_state = this.parse_game_state.bind(this);
	this.get_game_state_from_server = this.get_game_state_from_server.bind(this);
	this.state = {board: []};
	this.get_game_state_from_server();
	this.server_url = "localhost:8000";
  	this.json_text = fetch('http://' + this.server_url + '/game_state').then(res => res.json());
  }

	create_empty_board() { 
        	let board = [];
        	let num_rows = 6;
        	let num_columns = 6;
        	for (let i = 0; i < num_rows; i++) {
                	let new_column = [];
                	for (let j = 0; j < num_columns; j++) {
                        	new_column.push(0);
                	}
                	board.push(new_column);
        	}
        	return board;
  	}


	parse_game_state(json) {
		if (json.hasOwnProperty('board_state')) { 
			this.setState({board: json['board_state']['board']});
		}
		else { 
			this.setState({board: this.create_empty_board()});
		}
	}

	get_game_state_from_server() {
		fetch("http://" + this.server_url + "/game_state").then(response => response.json()).then(data => this.parse_game_state(data));
		setTimeout(this.get_game_state_from_server, 2000);
	}
	

 render() {
    return (
    	   <div>
	   {this.state.board.map((row, index) => {return <BoardRow row={row} row_num={index} key={index} /> } )} 
	    </div>
    )
}
}

