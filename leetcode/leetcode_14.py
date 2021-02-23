class Solution:
    def longestCommonPrefix(self, strs: List[str]) -> str:
        if len(strs) == 0:
            return ""

        resLen = 0
        while True:
            if len(strs[0]) == resLen:
                return strs[0]
            curChar = strs[0][resLen]
            for i in range(1, len(strs)):
                if len(strs[i]) == resLen or strs[i][resLen] != curChar:
                    return strs[0][: resLen]
            resLen += 1
        return strs[0][: resLen]    
