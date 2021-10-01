class Solution:
    board_ = []
    word_ = ''
    rowNum_ = 0
    colNum_ = 0

    def exist(self, board: List[List[str]], word: str) -> bool:
        self.board_ = board
        self.word_ = word
        self.rowNum_ = len(board)
        self.colNum_ = len(board[0])

        for i in range(0, self.rowNum_):
            for j in range(0, self.colNum_):
                if board[i][j] == word[0]:
                    if self.dfs(0, i, j):
                        return True
        return False

    def dfs(self, strIdx: int, rowIdx: int, colIdx: int) -> bool:
        if self.board_[rowIdx][colIdx] != self.word_[strIdx]:
            return False
        if strIdx == len(self.word_) - 1:
            return True
        
        tempChar = self.board_[rowIdx][colIdx]
        self.board_[rowIdx][colIdx] = ''
        if  (rowIdx > 0 and self.dfs(strIdx + 1, rowIdx - 1, colIdx)) or \
            (rowIdx < self.rowNum_ - 1 and self.dfs(strIdx + 1, rowIdx + 1, colIdx)) or \
            (colIdx > 0 and self.dfs(strIdx + 1, rowIdx, colIdx - 1)) or \
            (colIdx < self.colNum_ - 1 and self.dfs(strIdx + 1, rowIdx, colIdx + 1)):
                return True
        
        self.board_[rowIdx][colIdx] = tempChar
        return False
