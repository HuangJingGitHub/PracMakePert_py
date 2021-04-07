class Solution:
    def multiply(self, num1: str, num2: str) -> str:
        len1 = len(num1)
        len2 = len(num2)
        resCh = ['0'] * (len1 + len2)

        for i in range(len2 - 1, -1, -1):
            for j in range(len1 - 1, -1, -1):
                temp = ord(resCh[i + j + 1]) - ord('0') + (ord(num2[i]) - ord('0')) * (ord(num1[j]) - ord('0'))
                resCh[i + j + 1] = chr(ord('0') + temp % 10)
                resCh[i + j] = chr(ord(resCh[i + j]) + temp // 10)
        
        idx = 0
        while idx < len(resCh) and resCh[idx] == '0':
            idx += 1
        if idx == len(resCh):
            return '0'
        
        res = ''
        for i in range(idx, len(resCh)):
            res += resCh[i]
        return res
