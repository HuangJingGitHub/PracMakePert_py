class Solution:
    def solve(self, board: List[List[str]]) -> None:
        """
        Do not return anything, modify board in-place instead.
        """
        for row in range(len(board)):
            for col in range(len(board[0])):
                if row == 0 or row == len(board) - 1 or col == 0 or col == len(board[0]) - 1:
                    self.DFS_BorderProcess(board, row, col)
    
        for row in range(len(board)):
            for col in range(len(board[0])):
                if board[row][col] == 'O':
                    board[row][col] = 'X'
                elif board[row][col] == 'V':
                    board[row][col] = 'O'

    def DFS_BorderProcess(self, board: List[List[str]], row: int, col: int) -> None:
        if row < 0 or row >= len(board) or col < 0 or col >= len(board[0]):
            return
        if board[row][col] != 'O':
            return
        board[row][col] = 'V'   # visited 'O' connected with border 'O'

        self.DFS_BorderProcess(board, row, col - 1)
        self.DFS_BorderProcess(board, row, col + 1)
        self.DFS_BorderProcess(board, row - 1, col)
        self.DFS_BorderProcess(board, row + 1, col)
