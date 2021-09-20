class Solution:
    def minDistance(self, word1: str, word2: str) -> int:
        n1 = len(word1)
        n2 = len(word2)

        dp = [[0 for i in range(0, n2 + 1)] for j in range(0, n1 + 1)]
        for i in range(0, n2 + 1):
            dp[0][i] = i
        for i in range(0, n1 + 1):
            dp[i][0] = i
        
        for i in range(1, n1 + 1):
            for j in range(1, n2 + 1):
                if word1[i - 1] == word2[j - 1]:
                    dp[i][j] = dp[i - 1][j - 1]
                else:
                    temp = min(dp[i - 1][j], dp[i][j - 1])
                    temp = min(temp, dp[i - 1][j - 1])
                    dp[i][j] = 1 + temp
        return dp[-1][-1]
