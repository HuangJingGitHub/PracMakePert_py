class Solution:
    def uniquePaths(self, m: int, n: int) -> int:
        dp = [0] * n
        dp[0] = 1
        
        for row in range(0, m):
            for col in range(1, n):
                dp[col] += dp[col - 1]
        return dp[-1]
