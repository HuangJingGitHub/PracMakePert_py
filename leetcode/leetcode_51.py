class Solution:
    def solveNQueens(self, n: int) -> List[List[str]]:
        res = []
        board = [['.' for i in range(n)] for j in range(n)]
        self.backtrace(board, 0, res)
        return res
    
    def backtrace(self, board: List[List[str]], row: int, res: List[str]) -> None:
        n = len(board)
        if row == n:
            curRes = []
            for boardRow in board:
                curRes.append("".join(boardRow))
            res.append(curRes)
            print(res)
            return
        
        for col in range(n):
            if not self.isValid(board, row, col):
                continue
            board[row][col] = 'Q'
            self.backtrace(board, row + 1, res)
            board[row][col] = '.'
    
    def isValid(self, board: List[List[str]], row: int, col: int) -> bool:
        for i in range(row):
            if board[i][col] == 'Q':
                return False

        n = len(board)
        i = row - 1
        j = col + 1
        while i >= 0 and j < n:
            if board[i][j] == 'Q':
                return False
            i -= 1
            j += 1
        
        i = row - 1
        j = col - 1
        while i >= 0 and j >= 0:
            if board[i][j] == 'Q':
                return False
            i -= 1
            j -= 1
        return True
