

"""Describe this class"""
class Move():

    def __init__(self, board_location=None, is_play = False, is_pass=False, is_resign=False, ):
        
        self.board_location = board_location
        self.is_play = is_play
        self.is_pass = is_pass
        self.is_resign = is_resign

    @classmethod
    def play(cls, board_location):
        self.board_location = board_location

    @classmethod
    def pass(cls):
        return Move(is_pass=True)

    @classmethod
    def resign(cls):
        return Move(is_resign=True)
