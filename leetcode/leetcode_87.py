class Solution:
    def isScramble(self, s1: str, s2: str) -> bool:
        n = len(s1)
        m = len(s2)

        if n != m:
            return False
        
        tmp = [[False for i in range(n + 1)] for j in range(n)]
        dp = [copy.deepcopy(tmp) for i in range(n)]

        for i in range(n):
            for j in range(n):
                if s1[i] == s2[j]:
                    dp[i][j][1] = True
                else:
                    dp[i][j][1] = False 

        for l in range(2, n + 1):
            for i in range(n - l + 1):
                for j in range(n - l + 1):
                    for k in range(1, l):
                        if dp[i][j][k] and dp[i + k][j + k][l - k]:
                            dp[i][j][l] = True
                            break
                        elif dp[i][j + l - k][k] and dp[i + k][j][l - k]:
                            dp[i][j][l] = True
                            break
        return dp[0][0][n]
