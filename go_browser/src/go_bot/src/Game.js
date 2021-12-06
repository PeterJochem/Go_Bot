import logo from './go_bot5.gif';
import './App.css';
import React from 'react';
import Button from '@mui/material/Button';
import Paper from '@material-ui/core/Paper';
import BoardRow from "./board_row.js";

export default class Game extends React.Component {
  constructor(props) {
	super(props);
	this.render = this.render.bind(this)
	this.state = {button_name: this.props.name};
	this.server_url = "localhost:8000";
  	this.json_text = fetch('http://' + this.server_url + '/game_state').then(res => res.json());
  		
	
	this.board = [];
  	let num_rows = 6;
	let num_columns = 6;
	for (let i = 0; i < num_rows; i++) { 
		let new_column = [];
		for (let j = 0; j < num_columns; j++) {
			new_column.push(i * i + j % 3);
		}
		this.board.push(new_column);
	}
  
	  console.log(this.board);
  }

        componentDidMount() {
        	this.data = fetch("http://" + this.server_url + "/game_state").then(response => response.json()).then(data => (this.setState({button_name: data.game_running})));
	}

	
 render() {
    return (
    	   <div>
	    
	   {this.board.map((row, row_num) => {return <BoardRow row={row} row_num={row_num} key={row_num} /> } )} 
 			
	    </div>
    )
}
}

