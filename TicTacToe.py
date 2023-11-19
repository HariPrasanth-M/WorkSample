""" Tic-Tac-Toe game development - using class"""

from random import randrange


class TicTacToe:
    """Class for TicTacToe game."""

    def __init__(self):
        """ Initialize the required attributes"""

        self.winning_sets = [
            {0, 1, 2}, {0, 3, 6}, {0, 4, 8},
            {1, 4, 7}, {2, 5, 8}, {3, 4, 5},
            {2, 4, 6}, {6, 7, 8}
        ]

        self.space = ' '
        self.matrix = [
            self.space, self.space, self.space,
            self.space, self.space, self.space,
            self.space, self.space, self.space,
        ]

        self.comp_set = set({})
        self.user_set = set({})
        self.user_symbol = ' '
        self.computer_symbol = ' '

    def result(self):
        """ Comparing XO placements with winning combinations
        and display the winner.
        """

        for combinations in self.winning_sets:
            if self.user_set.issuperset(combinations):
                print("YOU WON!!!")
                return False
            if self.comp_set.issuperset(combinations):
                print("The computer has beaten you! You lose.")
                return False
        else:
            if self.space not in self.matrix:
                print("MATCH DRAW!")
                return False
        return True

    def print_pattern(self):
        """Print 3x3 matrix box with X & O placements"""

        print(
            f'{self.matrix[0]} | {self.matrix[1]} | {self.matrix[2]}\n- + - + -\n'
            f'{self.matrix[3]} | {self.matrix[4]} | {self.matrix[5]}\n- + - + -\n'
            f'{self.matrix[6]} | {self.matrix[7]} | {self.matrix[8]}\n'
        )

    def load_spot(self, position, player):
        """Load the spot with symbol according to player"""
        
        if self.matrix[position] == self.space:
            if player == 'computer':
                symbol = self.computer_symbol
                self.comp_set.add(position)
            else:
                symbol = self.user_symbol
                self.user_set.add(position)

            self.matrix[position] = symbol
            
            return True

        return False

    def get_input_and_validate(self):
        """Gets input from player and validate after validation"""

        input_num = ' '
        while True:
            input_num = input("What is your next move? (1-9)\n>>> ")
            if input_num.isdigit():
                input_num = int(input_num) - 1
                if (0 <= input_num <= 8
                        and self.matrix[input_num] == self.space):
                    break
                print("Enter a valid input which is not occupied")
            else:
                print("Enter a valid input")

        return input_num
    
    def get_winning_spot(self):
        """This returns the position for computer's turn"""

        comp_input = 0
        for i in self.winning_sets:
            comp_win_spot = i - self.comp_set

            if len(comp_win_spot) == 1:
                comp_input = comp_win_spot.pop()
                if self.matrix[comp_input] == self.space:
                    return comp_input

        for i in self.winning_sets:
            user_win_spot = i - self.user_set

            if len(user_win_spot) == 1:
                comp_input = user_win_spot.pop()
                if self.matrix[comp_input] == self.space:
                    break
        else:
            comp_input = randrange(0, 9)

        return comp_input

    def computer_turn(self):
        """Play Computer's turn using random module"""

        if self.user_symbol == 'X':
            self.computer_symbol = 'O'
        else:
            self.computer_symbol = 'X'

        while True:
            computer_input_position = self.get_winning_spot()
            
            if self.load_spot(computer_input_position, 'computer'):
                break

    def get_user_symbol(self):
        """Get user symbol, either X or O"""

        print("\nWelcome to Tic-Tac-Toe!")
        while True:
            self.user_symbol = input("Do you want to be X or O?\n>>> ").upper()
            if self.user_symbol in ('X', 'O'):
                break

    def run(self):
        """ Runs Tic-Tac-Toe game"""

        self.get_user_symbol()
        print("The computer will go first.")
        self.computer_turn()
        self.print_pattern()
        while self.result():
            user_input_position = self.get_input_and_validate()

            self.load_spot(user_input_position, 'user')

            if self.space in self.matrix:
                self.computer_turn()

            self.print_pattern()


def play_again():
    """play again request to user."""

    replay = ' '
    while True:
        replay = input("Do you want to play again? (yes or no)\n>>> ")
        if replay in ('yes', 'no'):
            break
    return replay


def main():
    """ Tic-Tac-Toe game """

    replay = 'yes'
    while replay == 'yes':
        game = TicTacToe()
        game.run()
        replay = play_again()


if __name__ == '__main__':
    main()
