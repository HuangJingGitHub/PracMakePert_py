class Solution:
    def minWindow(self, s: str, t: str) -> str:
        tStrLog = {}
        for c in t:
            if c not in tStrLog:
                tStrLog[c] = 1
            else:
                tStrLog[c] += 1


        charLog = {}
        for k, v in tStrLog.items():
            charLog[k] = [0 for i in range(0, len(s) + 1)]

        for i in range(0, len(s)):
            for k, v in charLog.items():
                charLog[k][i + 1] = charLog[k][i]
            if s[i] in charLog:
                charLog[s[i]][i + 1] += 1
        
        left = 0
        right = 0
        resLen = 1e10
        resLeft = 0
        
        while right < len(s):
            cover = True
            for k, v in tStrLog.items():
                if charLog[k][right + 1] - charLog[k][left] < v:
                    cover = False
                    break

            if cover:
                if right - left + 1 < resLen:
                    resLen = right - left + 1
                    resLeft = left
                left += 1
            else:
                right += 1

        if resLen == 1e10:
            return ''
        return s[resLeft : resLeft + resLen]
        
