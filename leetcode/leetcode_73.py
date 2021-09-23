class Solution:
    def setZeroes(self, matrix: List[List[int]]) -> None:
        """
        Do not return anything, modify matrix in-place instead.
        """
        rowZero = []
        colZero = []

        rowNum = len(matrix)
        colNum = len(matrix[0])
        for i in range(0, rowNum):
            for j in range(0, colNum):
                if matrix[i][j] == 0:
                    if i not in rowZero:
                        rowZero.append(i)
                    if j not in colZero:
                        colZero.append(j)
        
        for row in rowZero:
            for i in range(0, colNum):
                matrix[row][i] = 0
        for col in colZero:
            for i in range(0, rowNum):
                matrix[i][col] = 0
