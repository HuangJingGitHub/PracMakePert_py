class Solution:
    def plusOne(self, digits: List[int]) -> List[int]:
        if digits[-1] < 9:
            digits[-1] += 1
            return digits
        
        digits[-1] = 0
        for i in range(len(digits) - 2, -1, -1):
            digits[i] += 1
            if digits[i] <= 9:
                break
            digits[i] = 0
        
        if digits[0] != 0:
            return digits
        
        res = [1]
        for num in digits:
            res.append(num)
        return res
