class Solution:
    def isNumber(self, s: str) -> bool:
        idx = 0
        
        while idx < len(s) and s[idx] == ' ':
            idx += 1
        if idx == len(s):
            return False

        if s[idx] == '+' or s[idx] == '-':
            idx += 1

        digitNum = 0 
        pointNum = 0
        while idx < len(s) and ((s[idx] >= '0' and s[idx] <= '9') or s[idx] == '.'):
            if s[idx] == '.':
                pointNum += 1
            else:
                digitNum += 1
            idx += 1
        
        if pointNum > 1 or digitNum < 1:
            return False
        if idx == len(s):
            return True

        if s[idx] == 'e' or s[idx] == 'E':
            idx += 1
            if idx < len(s) and (s[idx] == '+' or s[idx] == '-'):
                idx += 1

            digitNum = 0
            while idx < len(s) and (s[idx] >= '0' and s[idx] <= '9'):
                idx += 1
                digitNum += 1
            if digitNum < 1:
                return False
        
        while idx < len(s) and s[idx] == ' ':
            idx += 1
        
        return idx == len(s)

