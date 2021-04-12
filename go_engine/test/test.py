#!/usr/bin/env python
#import roslib; 
#roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import sys
import unittest
import sys
sys.path.append("/home/peterjochem/Desktop/Go_Bot/catkin_ws/src/go_engine/src") # Use ROSlib
from deep_learning_go.go_types import BoardLocation
from deep_learning_go.go_string import GoString
from deep_learning_go.go_board import GoBoard
import rosunit

class Test_Go_String(unittest.TestCase):


    """Test GoBoard's  merge"""
    def test_legal_merging_groups(self):
        
        group_1_stones = [BoardLocation(3,3), BoardLocation(4,3)] 
        group_1_liberties = [BoardLocation(2,3), BoardLocation(5,3), BoardLocation(3,2), BoardLocation(3,4), BoardLocation(4,2), BoardLocation(4,4)] 

        group_2_stones = [BoardLocation(3,5), BoardLocation(4,5)]
        group_2_liberties = [BoardLocation(2,5), BoardLocation(5,5), BoardLocation(5,2), BoardLocation(5,4), BoardLocation(4,2), BoardLocation(4,4)]

        group1 = GoString(True, group_1_stones, group_1_liberties)
        group2 = GoString(True, group_2_stones, group_2_liberties)
        
        merged_group = group1.merge(group2)
        
        self.assertEqual(merged_group.get_num_stones(), 4)
        self.assertEqual(merged_group.get_num_liberties(), 10) # The two groups share some of the same liberties
        self.assertEqual(BoardLocation(3,3) in merged_group.stones, True)
        self.assertEqual(BoardLocation(4,3) in merged_group.stones, True)
        self.assertEqual(BoardLocation(3,5) in merged_group.stones, True)
        self.assertEqual(BoardLocation(4,5) in merged_group.stones, True)
        
    """Try to merge two groups of stones that are of different colors"""
    def test_illegal_merging_groups(self):

        group1 = GoString(True, [], [])
        group2 = GoString(False, [], [])
        
        merged_group = group1.merge(group2)
        isNone = (merged_group == None)
        self.assertEqual(isNone, True)
    
    """Add liberties to an existing GoString"""
    def test_add_liberty(self):

        group_1_stones = [BoardLocation(3,3), BoardLocation(4,3)]       
        group_1_liberties = [BoardLocation(2,3)]
        
        group1 = GoString(True, group_1_stones, group_1_liberties)
        group1.add_liberty(BoardLocation(3,4))
        
        self.assertEqual(group1.get_num_liberties(), 2)
        self.assertEqual(BoardLocation(3,4) in group1.liberties, True)
    
    """Remove liberties to an existing GoString"""
    def test_remove_liberty(self):
        group_1_stones = [BoardLocation(3,3), BoardLocation(4,3)]
        group_1_liberties = [BoardLocation(2,3)]

        group1 = GoString(True, group_1_stones, group_1_liberties)
        group1.remove_liberty(BoardLocation(2,3))

        self.assertEqual(group1.get_num_liberties(), 0)
        self.assertEqual(BoardLocation(2,3) in group1.liberties, False)

    
    """Add stones to an existing GoString"""
    def test_add_stone(self):

        group_1_stones = [BoardLocation(3,3), BoardLocation(4,3)]
        group_1_liberties = [BoardLocation(2,3), BoardLocation(4,4)]

        group1 = GoString(True, group_1_stones, group_1_liberties)
        group1.add_stone(BoardLocation(4,4))

        self.assertEqual(group1.get_num_stones(), 3)
        self.assertEqual(BoardLocation(4,4) in group1.stones, True)

    """Remove stones from an existing GoString"""
    def test_remove_stone(self):

        group_1_stones = [BoardLocation(3,3), BoardLocation(4,3)]
        group_1_liberties = [BoardLocation(2,3)]

        group1 = GoString(True, group_1_stones, group_1_liberties)
        group1.remove_stone(BoardLocation(3,3))

        self.assertEqual(group1.get_num_stones(), 1)
        self.assertEqual(BoardLocation(3,3) not in group1.stones, True)

    def test_equal_operator(self):
        pass


class Test_Go_Board(unittest.TestCase):
    
    def test_is_move_legal(self):

        myBoard = GoBoard(9, 9)
        isLegal = myBoard.isMoveLegal(BoardLocation(3,3), True)
        self.assertEqual(isLegal, True)
        
        isLegal = myBoard.isMoveLegal(BoardLocation(10,3), True)
        self.assertEqual(isLegal, False)
        
        isLegal = myBoard.isMoveLegal(BoardLocation(-1,3), True)
        self.assertEqual(isLegal, False)
        
        isLegal = myBoard.isMoveLegal(BoardLocation(4,-1), True)
        self.assertEqual(isLegal, False)

        isLegal = myBoard.isMoveLegal(BoardLocation(4,50), True)
        self.assertEqual(isLegal, False)

        
    def test_isOnBoard(self):
        
        myBoard = GoBoard(9, 9)
        self.assertEqual(myBoard.isOnBoard(BoardLocation(4,4)), True) 
        self.assertEqual(myBoard.isOnBoard(BoardLocation(0,0)), True)
        self.assertEqual(myBoard.isOnBoard(BoardLocation(8,8)), True)   

        self.assertEqual(myBoard.isOnBoard(BoardLocation(-1,4)), False)
        self.assertEqual(myBoard.isOnBoard(BoardLocation(4,-1)), False)
        self.assertEqual(myBoard.isOnBoard(BoardLocation(50,2)), False)   
        self.assertEqual(myBoard.isOnBoard(BoardLocation(6,100)), False)   


    def test_legal_place_stone(self):
        
        myBoard = GoBoard(9, 9)
         
        did_place = myBoard.place_stone(BoardLocation(8,8), True)
        self.assertEqual(did_place, True)
        self.assertEqual(myBoard.isEmpty(BoardLocation(8,8)), False)
    
    def test_illegal_place_stone(self):

        myBoard = GoBoard(9, 9)

        did_place = myBoard.place_stone(BoardLocation(10,8), True)
        self.assertEqual(did_place, False)
        did_place = myBoard.place_stone(BoardLocation(9,8), True)
        self.assertEqual(did_place, False)
        did_place = myBoard.place_stone(BoardLocation(-1,8), True)
        self.assertEqual(did_place, False)
        
        # Try to place a stone twice 
        did_place = myBoard.place_stone(BoardLocation(8,8), True)
        self.assertEqual(did_place, True)
        did_place = myBoard.place_stone(BoardLocation(8,8), True)
        self.assertEqual(did_place, False)

    
    def test_get_liberties(self):

        myBoard = GoBoard(9, 9)

        # Compute the liberties on the center of the empty board
        liberties = myBoard.get_liberties(BoardLocation(4,4))
        self.assertEqual(len(liberties), 4)

        # Compute the liberties on the edge of the board
        liberties = myBoard.get_liberties(BoardLocation(8,8))
        self.assertEqual(len(liberties), 2)


        # Place a piece and then compute the liberties 
        did_place = myBoard.place_stone(BoardLocation(4,4), True)
        liberties = myBoard.get_liberties(BoardLocation(4,3))
        self.assertEqual(len(liberties), 3)
    

    def test_illegal_merge_groups(self): 
        
        myBoard = GoBoard(9, 9)
        merged_group = myBoard.merge_groups([])
        isNone = (merged_group == None)
        self.assertEqual(isNone, True)


    def test_legal_merge_groups(self):
    
        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(1,1), BoardLocation(1,2)]
        g1_liberties = [BoardLocation(0,0)]
        
        g2_stones = [BoardLocation(1,3)]
        g2_liberties = [BoardLocation(0,3)]

        g3_stones = [BoardLocation(1,5)]
        g3_liberties = [BoardLocation(2,5)]
        

        groups = [GoString(True, g1_stones, g1_liberties), GoString(True, g2_stones, g2_liberties), GoString(True, g3_stones, g3_liberties)]
        merged_group = myBoard.merge_groups(groups)
        
        # Check that all three have their items in the merged_group
        self.assertEqual(len(merged_group.stones), 4)
        self.assertEqual(len(merged_group.liberties), 3)
        self.assertEqual(BoardLocation(1,1) in merged_group.stones, True)
        self.assertEqual(BoardLocation(1,2) in merged_group.stones, True)
        self.assertEqual(BoardLocation(1,3) in merged_group.stones, True)
        self.assertEqual(BoardLocation(1,5) in merged_group.stones, True)
        
        self.assertEqual(BoardLocation(0,0) in merged_group.liberties, True)
        self.assertEqual(BoardLocation(0,3) in merged_group.liberties, True)
        self.assertEqual(BoardLocation(2,5) in merged_group.liberties, True)
        
    
    def test_detect_need_to_merge_groups(self):
        
        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(1,1), BoardLocation(1,2)]
        g1_liberties = [BoardLocation(0,0)]

        g2_stones = [BoardLocation(1,3)]
        g2_liberties = [BoardLocation(0,3), BoardLocation(0,0)]

        g3_stones = [BoardLocation(1,5)]
        g3_liberties = [BoardLocation(2,5), BoardLocation(0,0)]


        groups = [GoString(True, g1_stones, g1_liberties), GoString(True, g2_stones, g2_liberties), GoString(True, g3_stones, g3_liberties)]
        myBoard.groups = groups

        # Detect friendly neighbors of g1
        groups = myBoard.neighboring_friendly_groups(BoardLocation(0,0), True)
        self.assertEqual(len(groups), 3)
         
        groups = myBoard.neighboring_friendly_groups(BoardLocation(0,0), False)
        self.assertEqual(len(groups), 0) 
        
    
    def test_remove_groups(self):

        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(1,1), BoardLocation(1,2)]
        g1_liberties = [BoardLocation(0,0)]
        
        g1 = GoString(True, g1_stones, g1_liberties)
        myBoard.groups = [g1]
        self.assertEqual(len(myBoard.groups), 1)
        myBoard.remove_group(g1)
        self.assertEqual(len(myBoard.groups), 0)
    
    
    def test_add_stone_to_group(self):

        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(1,1), BoardLocation(1,2)]
        g1_liberties = [BoardLocation(0,0)]
        
        g1 = GoString(True, g1_stones, g1_liberties)
        myBoard.groups = [g1]

        myBoard.add_stone_to_group(BoardLocation(0,0), True, g1)
        self.assertEqual(len(myBoard.groups), 1)
        self.assertEqual(g1.get_num_stones(), 3)
        self.assertEqual(g1.get_num_liberties(), 2)

        g2_stones = [BoardLocation(5,5), BoardLocation(5,4)]
        g2_liberties = [BoardLocation(5,3)]
        
        g2 = GoString(True, g2_stones, g2_liberties)
        myBoard.groups.append(g2)
        
        myBoard.add_stone_to_group(BoardLocation(5,3), True, g2)
        self.assertEqual(len(myBoard.groups), 2)
        self.assertEqual(g2.get_num_stones(), 3)
        self.assertEqual(g2.get_num_liberties(), 3)
        
    def test_add_illegal_stone_to_group(self):

        myBoard = GoBoard(9, 9)
        
        g1_stones = [BoardLocation(1,1), BoardLocation(1,2)]
        g1_liberties = [BoardLocation(0,0)]
        
        g1 = GoString(True, g1_stones, g1_liberties)
        myBoard.groups = [g1]
        
        try:
            myBoard.add_stone_to_group(BoardLocation(0,5), True, g1)
        except:
            pass    
        self.assertEqual(g1.get_num_stones(), 3)
        

    
    def test_place_stone_empty_board(self):
        
        myBoard = GoBoard(9, 9)

        myBoard.place_stone(BoardLocation(5,5), True)
        self.assertEqual(len(myBoard.groups), 1)
        self.assertEqual(myBoard.groups[0].get_num_stones(), 1)
        self.assertEqual(myBoard.groups[0].get_num_liberties(), 4)
    
    def test_place_stone_full_board_create_new_group1(self):

        myBoard = GoBoard(9, 9)
        
        g1_stones = [BoardLocation(1,1), BoardLocation(1,2)]
        g1_liberties = [BoardLocation(0,0), BoardLocation(0,1)]

        g1 = GoString(True, g1_stones, g1_liberties)
        myBoard.groups = [g1]
          
        myBoard.place_stone(BoardLocation(5,5), True)
        self.assertEqual(len(myBoard.groups), 2)
        self.assertEqual(g1.get_num_liberties(), 2)

        myBoard.place_stone(BoardLocation(7,7), True)
        self.assertEqual(len(myBoard.groups), 3)
        
        myBoard.place_stone(BoardLocation(0,0), False)
        self.assertEqual(len(myBoard.groups), 4)
        self.assertEqual(g1.get_num_liberties(), 1)
    
    """Place 1 black piece. Place 4 white pieces around it to capture it."""
    def test_place_stone_full_board_create_new_group2(self):

        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(5,5)]
        g1_liberties = [BoardLocation(5,4), BoardLocation(4,5), BoardLocation(5,6), BoardLocation(6,5)]

        g1 = GoString(True, g1_stones, g1_liberties)
        myBoard.groups = [g1]

        myBoard.place_stone(BoardLocation(5,4), False)
        self.assertEqual(len(myBoard.groups), 2)
        self.assertEqual(g1.get_num_liberties(), 3)

        myBoard.place_stone(BoardLocation(4,5), False)
        self.assertEqual(len(myBoard.groups), 3)

        myBoard.place_stone(BoardLocation(5,6), False)
        self.assertEqual(len(myBoard.groups), 4)
        self.assertEqual(g1.get_num_liberties(), 1)
        
        myBoard.place_stone(BoardLocation(6,5), False)
        self.assertEqual(len(myBoard.groups), 5)
        self.assertEqual(g1.get_num_liberties(), 0)

        
    """Place 1 black piece. Place 4 white pieces around it to capture it."""
    def test_place_stone_full_board_create_new_group2(self):

        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(5,5)]
        g1_liberties = [BoardLocation(5,4), BoardLocation(4,5), BoardLocation(5,6), BoardLocation(6,5)]

        g1 = GoString(True, g1_stones, g1_liberties)
        myBoard.groups = [g1]

        myBoard.place_stone(BoardLocation(5,4), False)
        self.assertEqual(len(myBoard.groups), 2)
        self.assertEqual(g1.get_num_liberties(), 3)

        myBoard.place_stone(BoardLocation(4,5), False)
        self.assertEqual(len(myBoard.groups), 3)

        myBoard.place_stone(BoardLocation(5,6), False)
        self.assertEqual(len(myBoard.groups), 4)
        self.assertEqual(g1.get_num_liberties(), 1)

        myBoard.place_stone(BoardLocation(6,5), False)
        self.assertEqual(len(myBoard.groups), 4)
        self.assertEqual(g1.get_num_liberties(), 0)

    """Create two black groups of stones and then play a move that requires merging them"""
    def test_place_stone_full_board_merge_groups(self):

        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(5,5)]
        g1_liberties = [BoardLocation(5,4), BoardLocation(4,5), BoardLocation(5,6), BoardLocation(6,5)]

        g2_stones = [BoardLocation(3,5)]
        g2_liberties = [BoardLocation(4,5), BoardLocation(2,5), BoardLocation(3,6), BoardLocation(3,4)]

        myBoard.groups = [GoString(True, g1_stones, g1_liberties), GoString(True, g2_stones, g2_liberties)]
        
        myBoard.place_stone(BoardLocation(4,5), True)
        self.assertEqual(len(myBoard.groups), 1)
        self.assertEqual(myBoard.groups[0].get_num_liberties(), 7)   
        self.assertEqual(myBoard.groups[0].get_num_stones(), 2)
        
    """Create two black groups of stones and then play a white piece on their shared liberty"""
    def test_place_stone_full_board_merge_groups(self):

        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(5,5)]
        g1_liberties = [BoardLocation(5,4), BoardLocation(4,5), BoardLocation(5,6), BoardLocation(6,5)]

        g2_stones = [BoardLocation(3,5)]
        g2_liberties = [BoardLocation(4,5), BoardLocation(2,5), BoardLocation(3,6), BoardLocation(3,4)]

        myBoard.groups = [GoString(True, g1_stones, g1_liberties), GoString(True, g2_stones, g2_liberties)]

        myBoard.place_stone(BoardLocation(4,5), False)
        self.assertEqual(len(myBoard.groups), 3)
        
    
    """Play a black stone. Then play 4 white stones to capture it"""
    def test_place_stone_that_captures_group(self):

        myBoard = GoBoard(9, 9)

        g1_stones = [BoardLocation(5,5)]
        g1_liberties = [BoardLocation(5,4), BoardLocation(4,5), BoardLocation(5,6), BoardLocation(6,5)]

        myBoard.groups = [GoString(True, g1_stones, g1_liberties)]

        myBoard.place_stone(BoardLocation(5,4), False)
        myBoard.place_stone(BoardLocation(4,5), False)
        myBoard.place_stone(BoardLocation(5,6), False)
        myBoard.place_stone(BoardLocation(6,5), False)

        self.assertEqual(len(myBoard.groups), 4)
    
    """Play a few black stones. Then play white stones to capture the group of black stones"""
    def test_place_stone_that_captures_group(self):

        myBoard = GoBoard(9, 9)

        myBoard.place_stone(BoardLocation(5,5), True)
        myBoard.place_stone(BoardLocation(5,6), True)
        myBoard.place_stone(BoardLocation(4,5), True)
        myBoard.place_stone(BoardLocation(5,4), True)
        myBoard.place_stone(BoardLocation(6,5), True)
        
        self.assertEqual(len(myBoard.groups), 1)
        self.assertEqual(myBoard.groups[0].get_num_liberties(), 8)

        myBoard.place_stone(BoardLocation(7,5), False)
        myBoard.place_stone(BoardLocation(5,3), False)
        myBoard.place_stone(BoardLocation(3,5), False)
        myBoard.place_stone(BoardLocation(5,7), False)
        
        myBoard.place_stone(BoardLocation(4,4), False)
        myBoard.place_stone(BoardLocation(6,6), False)
        myBoard.place_stone(BoardLocation(4,6), False)
        myBoard.place_stone(BoardLocation(6,4), False)

        self.assertEqual(len(myBoard.groups), 8)
        num_black_groups = 0
        for group in myBoard.groups:
            if (group.isBlack):
                num_black_groups = num_black_groups + 1
        self.assertEqual(num_black_groups, 0)
    

    """Describe me"""
    def test_capturing_group(self):         
        
        myBoard = GoBoard(3, 3)
        
        myBoard.place_stone(BoardLocation(1,0), False)
        myBoard.place_stone(BoardLocation(3,0), False)
        myBoard.place_stone(BoardLocation(2,0), False)
        
        myBoard.place_stone(BoardLocation(0,1), True)
        myBoard.place_stone(BoardLocation(1,0), True)

        self.assertEqual(len(myBoard.groups), 2)
        
        myBoard.place_stone(BoardLocation(2,1), True)
        self.assertEqual(len(myBoard.groups), 3) 

    """Describe me"""
    def test_capturing_group(self):

        myBoard = GoBoard(3, 3)

        myBoard.place_stone(BoardLocation(0,0), False)
        myBoard.place_stone(BoardLocation(0,2), True)
        myBoard.place_stone(BoardLocation(2,0), False)
        
        self.assertEqual(myBoard.groups[0].get_num_liberties(), 2) # 2 liberties 
        self.assertEqual(myBoard.groups[1].get_num_liberties(), 2) # 2 liberties
        self.assertEqual(myBoard.groups[2].get_num_liberties(), 2) # 2 liberties

        myBoard.place_stone(BoardLocation(0,1), True)
        
        
        
        myBoard.place_stone(BoardLocation(1,0), False)
        
        self.assertEqual(len(myBoard.groups), 2)
        
        self.assertEqual(myBoard.groups[0].get_num_liberties(), 2) # 2 liberties 
        self.assertEqual(myBoard.groups[1].get_num_liberties(), 2) # 1 liberties, but should be 2


        # This causes a capture of a group but it should not
        myBoard.place_stone(BoardLocation(2,1), True)
        #self.assertEqual(len(myBoard.groups), 3)


        


    """Apply a move and then undo a move to make sure the Zobrist hash remains the same"""
    def test_zobrist_hashing(self):    
        
        myBoard = GoBoard(9, 9)

        original_hash = myBoard.zobrist_hashes.current_hash
        myBoard.place_stone(BoardLocation(5,5), True)
        self.assertNotEqual(myBoard.zobrist_hashes.current_hash, original_hash)
        myBoard.remove_group(myBoard.groups[0])
        self.assertEqual(myBoard.zobrist_hashes.current_hash, original_hash)
    
    

     

    
    """
    def test_is_empty(self):

        myBoard = GoBoard(9, 9)
         
        self.assertEqual(myBoard.isEmpty(BoardLocation(3,4)), True)
    """ 





if __name__ == '__main__':
    rosunit.unitrun('go_engine', 'test_go_engine', Test_Go_String)
    rosunit.unitrun('go_engine', 'test_go_engine', Test_Go_Board)


#if __name__ == '__main__':
#    unittest.main()
