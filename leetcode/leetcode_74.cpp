class Solution:
    def searchMatrix(self, matrix: List[List[int]], target: int) -> bool:
        rowNum = len(matrix)
        colNum = len(matrix[0])

        left = 0
        right = rowNum * colNum - 1
        while left <= right:
            mid = left + (right - left) // 2
            curRow = mid // colNum
            curCol = mid % colNum
            if matrix[curRow][curCol] == target:
                return True
            elif matrix[curRow][curCol] < target:
                left = mid + 1
            else:
                right = mid - 1
        return False
