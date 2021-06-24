class Solution:
    def uniquePathsWithObstacles(self, obstacleGrid: List[List[int]]) -> int:
        m = len(obstacleGrid)
        n = len(obstacleGrid[0])
        dp = [[0 for i in range(n)] for j in range(m)]

        if obstacleGrid[0][0] == 1:
            return 0
        
        dp[0][0] = 1
        for col in range(1, n):
            if obstacleGrid[0][col] == 1:
                dp[0][col] = 0
            else:
                dp[0][col] = dp[0][col - 1]
        for row in range(1, m):
            if obstacleGrid[row][0] == 1:
                dp[row][0] = 0
            else:
                dp[row][0] = dp[row - 1][0]
        
        for row in range(1, m):
            for col in range(1, n):
                if obstacleGrid[row][col] == 1:
                    dp[row][col] = 0
                else:
                    dp[row][col] = dp[row][col - 1] + dp[row - 1][col]
        return dp[-1][-1]
