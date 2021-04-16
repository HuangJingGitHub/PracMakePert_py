class Solution:
    def isMatch(self, s: str, p: str) -> bool:
        if len(s) == 0:
            if len(p) == 0:
                return True
            else:
                for ch in p:
                    if ch != '*':
                        return False
                return True
        if len(p) == 0:
            return False
        
        dp = [[False for i in range(len(p))] for j in range(len(s))]

        if s[0] == p[0] or p[0] == '?' or p[0] == '*':
            dp[0][0] = True
        if p[0] == '*':
            for i in range(1, len(s)):
                dp[i][0] = True
        
        cnt = 0
        if p[0] == '?' or p[0] == s[0]:
            cnt = 1

        for i in range(1, len(p)):
            if dp[0][i - 1]:
                if p[i] == '*':
                    dp[0][i] = True
                elif p[i] == '?' or p[i] == s[0]:
                    if cnt == 0:
                        dp[0][i] = True
                        cnt += 1
        
        for i in range(1, len(s)):
            for j in range(1, len(p)):
                if dp[i - 1][j - 1]:
                    if s[i] == p[j] or p[j] == '?' or p[j] == '*':
                        dp[i][j] = True
                if dp[i - 1][j]:
                    if p[j] == '*':
                        dp[i][j] = True
                if dp[i][j - 1]:
                    if p[j] == '*':
                        dp[i][j] = True
        return dp[-1][-1]
