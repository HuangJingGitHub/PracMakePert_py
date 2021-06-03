class Solution:
    def generateMatrix(self, n: int) -> List[List[int]]:
        res = [[1 for i in range(n)] for j in range(n)]

        num = 1
        for i in range(0, n // 2 + 1):
            for col in range(i, n - i):
                res[i][col] = num
                num += 1
            for row in range(i + 1, n - i):
                res[row][n - i - 1] = num
                num += 1
            for col in range(n - i - 2, i - 1, -1):
                res[n - i - 1][col] = num
                num += 1
            for row in range(n - i - 2, i, -1):
                res[row][i] = num
                num += 1
        return res
