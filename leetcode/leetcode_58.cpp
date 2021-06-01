class Solution:
    def lengthOfLastWord(self, s: str) -> int:
        res = 0

        endIdx = -1
        for i in range(len(s) - 1, -1, -1):
            if s[i] == ' ':
                continue
            else:
                endIdx = i
                break

        if endIdx == -1:
            return 0

        startIdx = -1
        for i in range(endIdx - 1, -1, -1):
            if s[i] == ' ':
                startIdx = i
                break
            else:
                continue
        
        return endIdx - startIdx

# more elegant
class Solution:
    def lengthOfLastWord(self, s: str) -> int:
        res = 0

        for c in reversed(s):
            if c == ' ' and res == 0:
                continue
            elif c == ' ' and res != 0:
                return res
            res += 1
        
        return res        
