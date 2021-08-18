class Solution:
    def addBinary(self, a: str, b: str) -> str:
        res = ''
        aLen = len(a)
        bLen = len(b)
        resLen = max(aLen, bLen)
        resChar = ['0' for i in range(resLen)]

        carry = False
        for i in range(resLen):
            aChar = '0'
            bChar = '0'
            if i < aLen:
                aChar = a[aLen - 1 - i]
            if i < bLen:
                bChar = b[bLen - 1 - i]
            sum = ord(aChar) - ord('0') + ord(bChar) - ord('0')
            if carry:
                sum += 1
                carry = False
            
            if sum <= 1:
                resChar[resLen - 1 - i] = chr(sum + ord('0'))
            elif sum == 2:
                resChar[resLen - 1 - i] = '0'
                carry = True
            else:
                resChar[resLen - 1 - i] = '1'
                carry = True
        
        if carry:
            res = '1'
        for ch in resChar:
            res += ch
        return res

