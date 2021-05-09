from go_types import BoardLocation
from go_string import GoString
from zobrist_hash import ZobristHash


""" """
class GoBoard():

    def __init__(self, num_rows, num_columns):
        
        self.num_rows = num_rows
        self.num_columns = num_columns
        self.groups = [] # List of GoStrings
        self.zobrist_hashes = ZobristHash(num_rows, num_columns)
        self.previous_states = {}
        self.num_black_captured_pieces = 0
        self.num_white_captured_pieces = 0
                  

    """Describe Me"""
    def place_stone(self, board_location, isBlack):
        
        if (self.isMoveLegal(board_location, isBlack) == False):
            print("Place stone called with a move which is not legal") # Throw an error instead
            print(str(board_location.row) + "     " + str(board_location.column)) 
            return None

        neighboring_friendly_groups = self.neighboring_friendly_groups(board_location, isBlack)
        if (len(neighboring_friendly_groups) > 1):
            # Merge a few groups into one
            self.create_one_group(neighboring_friendly_groups, board_location, isBlack)

        elif(len(neighboring_friendly_groups) == 1):
            # Add the stone to the existing group
            self.add_stone_to_group(board_location, isBlack, neighboring_friendly_groups[0])
        else:
            self.add_group(GoString(isBlack, [board_location], self.get_liberties(board_location))) 
        
        # Update any neighboring enemy groups
        neighboring_enemy_groups = self.neighboring_enemy_groups(board_location, isBlack)
        for i in range(len(neighboring_enemy_groups)):
            group = neighboring_enemy_groups[i]
            group.remove_liberty(board_location)
             
        return self.remove_captured_groups()  
        #return True
    
    
    def create_one_group(self, existing_friendly_groups, linking_board_location, isBlack):

        # Merge Groups
        new_group = GoString(isBlack, [linking_board_location], self.get_liberties(linking_board_location))
        self.groups.append(new_group)
        existing_friendly_groups.append(new_group)

        merged_group = self.merge_groups(existing_friendly_groups)
        
        # Delete the old groups
        for i in range(len(existing_friendly_groups)):
            self.remove_group(existing_friendly_groups[i])

        self.add_group(merged_group)



    def update_previous_state(self):
        new_hash = self.zobrist_hashes.apply_move(board_location, isBlack)
        self.previous_states = self.previous_states + new_hash

    
    # FIX ME - re-write this method
    def remove_captured_groups(self):
        #self.groups = [group for group in self.groups if group.get_num_liberties() >= 1]    
 
        captured_groups = []

        i = 0
        while (i < len(self.groups)):
            if (self.groups[i].get_num_liberties() <= 0):
                print("removing a group because it was captured")
                captured_group = self.groups[i] 
                self.remove_group(captured_group)
                captured_groups.extend(captured_group.stones)

                # Must update the existing groups to now have more liberties
                for j in range(len(self.groups)):
                    existing_group = self.groups[j]
                    if (existing_group.isBlack != captured_group.isBlack):

                        for stone in existing_group.stones:
                            for neighbor in stone.neighbors():
                                if (neighbor in captured_group.stones):
                                    existing_group.liberties.add(neighbor)

            i = i + 1
        
        return captured_groups
        

    def add_stone_to_group(self, new_board_location, isBlack, existing_group):
        
        if (new_board_location not in existing_group.liberties or isBlack != existing_group.isBlack):
            # Throw an error instead
            print("Error. Trying to add a stone to a group which is not in the group's liberty list")
        
        existing_group.add_stone(new_board_location)
        existing_group.remove_liberty(new_board_location)
        for liberty in self.get_liberties(new_board_location):
            existing_group.add_liberty(liberty)
        
        self.zobrist_hashes.apply_move(new_board_location, isBlack)
    
    def isMoveLegal(self, board_location, isBlack):
        
        if (self.isOnBoard(board_location) == False or self.isEmpty(board_location) == False):
            return False
        
        # The move must respect Ko  
        if (self.zobrist_hashes.get_hash_value(board_location, isBlack) in self.previous_states):
            return False

        # The group which we add the new stone to must have more than 1 liberty AFTER placing the stone        
        neighoring_friendly_groups = self.neighboring_friendly_groups(board_location, isBlack) 
        merged_group_liberties = set()
        for group in neighoring_friendly_groups:
            merged_group_liberties = merged_group_liberties.union(set(group.liberties))

        merged_group_liberties = merged_group_liberties.union(set(self.get_liberties(board_location)))
        try:
            merged_group_liberties.remove(board_location)
        except:
            pass
        if (len(merged_group_liberties) == 0):
            return False
        
        return True
    

    def isOnBoard(self, board_location):

        if (board_location.row < 0 or board_location.row >= self.num_rows):
            return False
        if (board_location.column < 0 or board_location.column >= self.num_columns):
            return False
        return True

    def isEmpty(self, board_location):
        
        for group in self.groups:
            if (board_location in group.stones):
                return False
        
        return True

    def does_piece_exist_at_location(self, board_location, isBlack):
    
        for group in self.groups:
            if (group.isBlack == isBlack):
                for stone in group.stones:
                    if (stone == board_location):
                        return True

        return False



    def neighboring_friendly_groups(self, board_location, isBlack):
        
        neighboring_friendly_groups = []

        for group in self.groups:
            if (group.isBlack == isBlack and board_location in group.liberties):
                neighboring_friendly_groups.append(group)

        return neighboring_friendly_groups
    
    def neighboring_enemy_groups(self, board_location, isBlack):

        neighboring_enemy_groups = []

        for group in self.groups:
            if (group.isBlack != isBlack and board_location in group.liberties):
                neighboring_enemy_groups.append(group)

        return neighboring_enemy_groups


    def merge_groups(self, all_groups):
        
        if (all_groups == None or len(all_groups) == 0):
            print("Error. Merge groups called with a list of length 0")
        if (len(all_groups) == 1):
            print("Warning. merge_groups was called with a list of exactly one group")
            return all_groups[0]

        merged_group = None
        for i in range(len(all_groups)):
            new_group = all_groups[i]
            if (merged_group == None):
                merged_group = new_group
                continue

            merged_group = merged_group.merge(new_group) 
            
        return merged_group
            
    
    def remove_group(self, go_string):     
        self.groups.remove(go_string)
        for stone in go_string.stones:
            self.zobrist_hashes.undo_move(stone, go_string.isBlack)
        
        if (go_string.isBlack):
            self.num_black_captured_pieces += go_string.get_num_stones()
        else:
            self.num_white_captured_pieces += go_string.get_num_stones()


    def add_group(self, go_string):
        self.groups.append(go_string)
        for stone in go_string.stones:
            self.zobrist_hashes.undo_move(stone, go_string.isBlack)

    
    """Compute and return a list of liberties for the group with only one stone at the given board_location"""
    def get_liberties(self, board_location):
        
        liberties = []

        neighbors = board_location.neighbors()   
        for neighbor in neighbors:
            if (self.isOnBoard(neighbor) and self.isEmpty(neighbor)):
                liberties.append(neighbor)
                
        return liberties
        

