
server_url = "localhost:8000"

function get_game_state() {
	const Http = new XMLHttpRequest();
	const url='http://' + server_url + '/game_state';
	Http.open("GET", url);
	Http.send();

	Http.onreadystatechange = function() {
  		if (this.readyState==4 && this.status==200) {
  			console.log(Http.responseText)
	  	}
	}
}
