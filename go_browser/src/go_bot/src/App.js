import logo from './go_bot5.gif';
import './App.css';
import React from 'react';
import Button from '@mui/material/Button';
import Game from "./Game.js";
import PersistentDrawerRight from './drawer.js';

export default class App extends React.Component {
 render() {
    return (
	<div>
		<PersistentDrawerRight> </PersistentDrawerRight>
		<Game name=""> </Game>
	</div>
    );
}
}

