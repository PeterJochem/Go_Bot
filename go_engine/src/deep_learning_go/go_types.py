import enum

class Player(enum.Enum):

    black = 1
    white = 2

    @property
    def other(self):
        if (self == Player.white):
            return Player.white
        else:
            return Player.black

"""Describe me"""
class BoardLocation():
    def __init__(self, row, column):
        
        self.row = row
        self.column = column

    
    def neighbors(self):
        return [BoardLocation(self.row - 1, self.column), BoardLocation(self.row + 1, self.column), BoardLocation(self.row, self.column - 1), BoardLocation(self.row, self.column + 1)]

    def __eq__(self, other):
        if (self.row == other.row and self.column == other.column):
            return True
        return False

    def __hash__(self):
        a = 1828
        b = 321
        c = (19 * 19) * 8731
        return (hash(self.row) * a + hash(self.column) * b) % c 
