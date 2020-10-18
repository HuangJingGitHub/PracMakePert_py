# dp without optimization, will exceed time limit.
class Solution:
    def minCut(self, s: str) -> int:
        if len(s) < 2 or self.isPalindrome(s, 0, len(s) - 1):
            return 0

        dp = [0] * (len(s) + 1)
        dp[0] = -1
        for i in range(1, len(s)):
            dp[i + 1] = dp[i] + 1
            for j in range(i):
                if self.isPalindrome(s, j, i):
                    dp[i + 1] = min(dp[i + 1], dp[j] + 1)
        
        return dp[-1]
        
    def isPalindrome(self, s: str, start: int, end: int) -> bool:
        if start >= len(s) or end < 0 or start > end:
            return False
        
        while start < end:
            if s[start] != s[end]:
                return False
            start += 1
            end -= 1
        
        return True


# AC using dp with optimization on the palindrome check method. For details in this method, refer to Problem 5 solution discussion.
# But it can be seen from the code, also use dp for check.
class Solution:
    def minCut(self, s: str) -> int:
        size = len(s)
        if size < 2:
            return 0

        dp = [0] * (size + 1)
        dp[0] = -1

        isPalindrome = [[False for _ in range(size)] for _ in range(size)]
        for right in range(size):
            for left in range(right + 1):
                if s[left] == s[right] and (right - left <= 2 or isPalindrome[left + 1][right - 1]):
                     isPalindrome[left][right] = True


        for i in range(1, size):
            dp[i + 1] = dp[i] + 1
            for j in range(i):
                if isPalindrome[j][i]:
                    dp[i + 1] = min(dp[i + 1], dp[j] + 1)
        
        return dp[-1]        
