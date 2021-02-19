class Solution:
    def romanToInt(self, s: str) -> int:
        values = [1000, 900, 500, 400, 100, 90, 50, 40, 10, 9, 5, 4, 1]
        romanReps = ["M", "CM", "D", "CD", "C", "XC", "L", "XL", "X", "IX", "V", "IV", "I"]

        res = 0
        idx = 0
        for i in range(len(romanReps)):
            while (idx < len(s) and s[idx:idx + len(romanReps[i])] == romanReps[i]):
                res += values[i]
                idx += len(romanReps[i])
        return res        
