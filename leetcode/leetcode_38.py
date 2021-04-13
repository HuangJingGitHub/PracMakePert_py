class Solution:
    def countAndSay(self, n: int) -> str:
        if n == 1:
            return '1'

        lastStr = '1'
        curStr = ''

        for i in range(1, n):
            curStr = ''            
            lastChar = lastStr[0]
            cnt = 1
            for j in range(1, len(lastStr)):
                if lastStr[j] == lastChar:
                    cnt += 1
                else:
                    curStr += str(cnt) + lastChar
                    lastChar = lastStr[j]
                    cnt = 1
            curStr += str(cnt) + lastChar
            lastStr = curStr
        
        return curStr
