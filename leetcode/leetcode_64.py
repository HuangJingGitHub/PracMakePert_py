class Solution:
    def longestValidParentheses(self, s: str) -> int:
        if len(s) == 0:
            return 0

        dp = [0] * len(s)
        for i in range(1, len(s)):
            if s[i] == ')':
                testIdx = i - dp[i - 1] - 1
                if testIdx >= 0 and s[testIdx] == '(':
                    dp[i] = dp[i - 1] + 2
                    if testIdx > 0:
                        dp[i] += dp[testIdx - 1]
        return max(dp)
