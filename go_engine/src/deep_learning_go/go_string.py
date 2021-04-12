#import go_types
from go_types import BoardLocation

""" """
class GoString():

    def __init__(self, isBlack, stones, liberties):
        self.isBlack = isBlack
        self.stones = set(stones) # List of board locations
        self.liberties = set(liberties) # List of board locations


    def add_liberty(self, board_location):
        self.liberties.add(board_location)

    def remove_liberty(self, board_location):
        self.liberties.remove(board_location)
    
    def add_stone(self, board_location):
        self.stones.add(board_location)

    def remove_stone(self, board_location):
        self.stones.remove(board_location)

    def get_num_liberties(self):
        return len(self.liberties)
    
    def get_num_stones(self):
        return len(self.stones)
    
    def merge(self, other_go_string):

        if (self.isBlack != other_go_string.isBlack):
            print("Error. Trying to merge two go strings with different colors")
            return None

        all_stones = self.stones.union(other_go_string.stones)
        new_liberties = self.liberties.union(other_go_string.liberties) - all_stones

        return GoString(self.isBlack, all_stones, new_liberties)

    
    """
    def __eq__(self, other_go_string):

        if (isinstance(other_go_string, GoString) == False or self.isBlack != other_go_string.isBlack):
            return False
        if (self.stones != other_go_string.stones or self.liberties != other_go_string.liberties):
            return False

        return True
    """
