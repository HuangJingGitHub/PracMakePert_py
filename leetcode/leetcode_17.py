class Solution:
    def __init__(self):
        self.letterMap = ["abc", "def", "ghi", "jkl", "mno", "pqrs", "tuv", "wxyz"]
        self.res = []
        self.curStr = ""
        self.digits = ""
        
    def letterCombinations(self, digits: str) -> List[str]:
        if len(digits) == 0:
            return []
        
        self.digits = digits
        self.backtrace(0)
        return self.res

    def backtrace(self, idx):
        if idx == len(self.digits):
            self.res.append(self.curStr)
            return
        
        curDigit = ord(self.digits[idx]) - ord('2')
        buttonStr = self.letterMap[curDigit]

        for ch in buttonStr:
            self.curStr += ch
            self.backtrace(idx + 1)
            self.curStr = self.curStr[:-1]
