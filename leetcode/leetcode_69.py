class Solution:
    def mySqrt(self, x: int) -> int:
        if x == 0:
            return 0
        
        cur = 1.0
        pre = 1.0

        while True:
            pre = cur
            cur = cur / 2 + x / (2 * cur)
            if abs(pre - cur) < 0.1:
                return int(cur)
