class Solution:
    def searchMatrix(self, matrix: List[List[int]], target: int) -> bool:
        rows = len(matrix)
        cols = len(matrix[0])

        curRow = rows - 1
        curCol = 0

        while curRow >= 0 and curCol < cols:
            if matrix[curRow][curCol] == target:
                return True
            elif matrix[curRow][curCol] < target:
                curCol += 1
            else:
                curRow -= 1
        return False
