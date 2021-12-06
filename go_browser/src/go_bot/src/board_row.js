import React from 'react';
import { makeStyles } from '@material-ui/core/styles';
import Paper from '@material-ui/core/Paper';

const p1Color =  "#101010";
const p2Color = "white"; 
const emptyColor = "#E6BF83"; //"#E6BF83";


const useStyles = makeStyles((theme) => ({
  board_background: {
        backgroundColor: 'white',
	  display: 'flex',
    	justifyContent: 'center',
	  flexWrap: 'block',
    '& > *': {
      margin: theme.spacing(1),
      width: theme.spacing(16),
      height: theme.spacing(16),
    },
  },
}));

function get_color(row, column_num) {
	
	let color = emptyColor;
	if (row[column_num] === 1) {
		color = p1Color;
	}
	else if (row[column_num] === 2) {
		color = p2Color;
	}
	return color;
}


export default function BoardRow(props) {
  const classes = useStyles();
  
  return <div className={classes.board_background} >
      
      {props.row.map((value, column_num) => {return <Paper style={{backgroundColor: get_color(props.row, column_num), }} onClick={() => console.log(`${props.row_num}, ${column_num}`)} elevation={10} className={classes.board_location} /> 
        })
      }
      </div>
}
