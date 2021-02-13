# a dp method
class Solution:
    def longestPalindrome(self, s: str) -> str:
        size = len(s)
        if size < 2:
            return s
        
        dp = [[False for _ in range(size)] for _ in range(size)]
        maxLen = 1
        start = 0

        for i in range(size):
            dp[i][i] = True
        
        for j in range(1, size):
            for i in range(0, j):
                if s[i] == s[j]:
                    if j - i <= 2:
                        dp[i][j] = True
                    else:
                        dp[i][j] = dp[i + 1][j - 1]

                if dp[i][j]:
                    curLen = j - i + 1
                    if curLen > maxLen:
                        maxLen = curLen
                        start = i

        return s[start : start + maxLen]
                 
