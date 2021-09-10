
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

function start_game() {
        const Http = new XMLHttpRequest();
        const url='http://' + server_url + '/start_game?num_rows=22&num_columns=22&whitePlayerType=a&blackPlayerType=b';
        Http.open("POST", url);
        Http.onload = function() {
                if (this.readyState==4 && this.status==200) {
                        console.log(Http.responseText)
                }
        }
	Http.send();
}

