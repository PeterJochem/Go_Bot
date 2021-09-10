from typing import Optional
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from go_motion_planning.srv import *
from go_motion_planning.srv import pickup_set_of_pieces
from go_bot_server.srv import move #as ABC
import sys
sys.path.insert(0, "../../../go_engine/src/")
sys.path.insert(0, "../../../go_engine/src/deep_learning_go")
from deep_learning_go.game_state import GameState
from go_browser.msg import gamestate
import time
from go_types import BoardLocation
from go_board import PydanticGoBoard
from threading import Thread
from random_agent import RandomAgent
from online_agent import OnlineAgent
from deep_q_agent import DeepQLearningAgent


""" @brief Class for coordinating a game """
class go_commander:

    def __init__(self):

        self.is_game_running = False
        self.game_state = None

        self.delete_pieces_gazebo = rospy.ServiceProxy("/remove_pieces_from_gazebo", remove_pieces_from_gazebo)
        self.spawn_piece = rospy.ServiceProxy("/spawn_piece", spawn_piece)
        self.remove_piece = rospy.ServiceProxy("/remove_piece", remove_pieces_from_gazebo)
        self.pickup_piece = rospy.ServiceProxy("/pickup_piece", pickup_piece)
        self.place_piece = rospy.ServiceProxy("/place_piece", place_piece)
        self.pickup_set_of_pieces = rospy.ServiceProxy("/pickup_set_of_pieces", pickup_set_of_pieces)
        self.enter_human_black_move = rospy.ServiceProxy("/enter_online_player_move/black_player", move)
        self.enter_human_white_move = rospy.ServiceProxy("/enter_online_player_move/white_player", move)
        self.home_position = rospy.ServiceProxy("/home_position", home_position)
        self.rate = rospy.Rate(10)         
    
    def play_game(self):
        while (not self.game_state.isOver()):
            board_location = self.game_state.get_next_players_move()
            if (board_location is None):
                # The agent passed - no need to update the physical board
                self.game_state.execute_move(board_location)
                continue
            
            self.rate.sleep()
            self.update_real_board(board_location, self.game_state.blacksTurn)
            captured_pieces = self.game_state.execute_move(board_location)
            self.game_state.display_board()
            self.remove_captured_groups(captured_pieces)

            # You could also call a method here to let objects lower in tree spin once
            #self.game_state_publisher.publish(self.game_state.toMsg())
            self.rate.sleep()
    
    def init_game(self, num_rows, num_columns, whitePlayerType=RandomAgent, blackPlayerType=OnlineAgent):
        if (num_rows < 1 or num_columns < 1):
            return False
        
        if (not self.is_game_running):
            self.game_state = GameState(num_rows, num_columns, whitePlayerType, blackPlayerType)
            self.is_game_running = True
            return True
        else:
            return False

    def update_real_board(self, board_location, isBlack):

        try:
            # FIX ME - For now, spawn a piece at a hardcoded location
            self.spawn_piece(5, 5, isBlack)
            time.sleep(0.1)
            self.pickup_piece(5, 5)
            
            print("Board Location (row, column) = " + str(board_location.row) + ", " + str(board_location.column))
            self.place_piece(board_location.row, board_location.column)
        except:
            print("Caught an error when trying to update the real board in play_game.py")
            print("Board Location (row, column) = " + str(board_location.row) + ", " + str(board_location.column))

    def remove_captured_groups(self, captured_pieces):

        if (captured_pieces is None or len(captured_pieces) == 0):
            return

        row_cords, column_cords  = [], []
        for piece in captured_pieces:
            row_cords.append(piece.row)
            column_cords.append(piece.column)
        self.pickup_set_of_pieces(row_cords, column_cords)

    def create_game_state_JSON(self):
        
        if (self.game_state is None):
            return {"game_running": False}
        board_as_lists = self.game_state.create_list_representation()
        board = PydanticGoBoard(board=board_as_lists)
        isBlackTurn = self.game_state.blacksTurn
        white_territory = 0
        black_territory = 0
        return {"game_running": True, "board_state": board, "isBlackTurn": isBlackTurn, "white_territory": white_territory, 
                "black_territory": black_territory}
        
def parse_player_type(playerType: str):
    if playerType == "OnlineAgent":
        return OnlineAgent
    elif playerType == "RLAgent":
        return DeepQLearningAgent
    elif playerType == "RandomAgent":
        return RandomAgent
    else:
        return RandomAgent

app = FastAPI()
origins = [
    "http://localhost",
    "http://localhost:8000"
    "http://localhost:8080",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

rospy.init_node("server")
commander = go_commander()

@app.get("/game_state")
def get_game_state_JSON():
    """Returns JSON describing the current game's state"""
    try:
        return commander.create_game_state_JSON()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return {"success": False}

@app.post("/start_game")
def start_game(num_rows: int, num_columns: int, whitePlayerType: str, blackPlayerType: str):
    """Try to initialize a game. Returns True/False if a game is/is not possible"""
    try:
        whitePlayerType = parse_player_type(whitePlayerType) 
        blackPlayerType = parse_player_type(blackPlayerType)
        if not commander.init_game(num_rows, num_columns, whitePlayerType, blackPlayerType):
            return {"success": False}   
        game_thread = Thread(target=commander.play_game, args=())
        success = game_thread.start()
    except Exception as e:
        print("Could not start a game: %s"%e)
        return {"success": False}
    return {"success": True}

@app.post("/enter_online_player_move")
async def make_move(row: int, column: int, isBlack: bool):
    """Update the online players move. Does not check if move is legal""" 
    try:
        if (isBlack):
            commander.enter_human_black_move(row, column)
        else:
            commander.enter_human_white_move(row, column)
        return {"success": True}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return {"success": False}

@app.post("/home_position")
async def home_position():
    """Move the robot to its home position. Does not respect game context"""
    try:
        commander.home_position()
        return {"success": True}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return {"success": False}
       
@app.post("/place_piece")
async def place_piece(row: int, column: int, isBlackPlayer: bool):
    """Request the robot to place the piece. Does not respect game context"""
    try:
        commander.place_piece(row, column)
        return {"success": True}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return {"success": False}

@app.post("/pickup_piece")
async def pickup_piece(row: int, column: int):
    """Request the robot to pickup a piece at the given location. 
    Does not respect game context"""
    try:
        commander.pickup_piece(row, column)
        return {"success": True}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return {"success": False}
        
@app.post("/remove_piece")
async def remove_piece(row: int, column: int):
    """Request the robot to remove a piece at the given location.
    Does not respect game context"""
    try:
        commander.remove_piece(row, column)
        return {"success": True}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return {"success": False}

@app.post("/remove_all_pieces_gazebo")
async def remove_all_gazebo_pieces():
    """Remove all the pieces from Gazebo. Does not remove pieces from
    the current game. Does not respect game context"""
    try:
        commander.delete_pieces_gazebo()
        return {"success": True}
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return {"success": False}
