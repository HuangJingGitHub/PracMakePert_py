class Solution:
    def climbStairs(self, n: int) -> int:
        if n == 1:
            return 1
        if n == 2:
            return 2
        prePre = 1
        pre = 2
        cur = 0

        for i in range(3, n + 1):
            cur = prePre + pre
            prePre = pre
            pre = cur
        return cur
