class Solution:
    def lengthOfLongestSubstring(self, s: str) -> int:
        if len(s) <= 1:
            return len(s)

        res = 0
        left = 0
        right = 0
        for i in range(1, len(s)):
            for j in range(left, right + 1):
                if s[i] == s[j]:
                    res = max(res, right - left + 1)
                    left = j + 1
                    right = i
                    break
            right = i
        return max(res, right - left + 1)
