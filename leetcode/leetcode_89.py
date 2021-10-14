class Solution:
    def grayCode(self, n: int) -> List[int]:
        if n == 0:
            return [0]

        res = self.grayCode(n - 1)
        halfNum = 2 ** (n - 1)
        for i in range(halfNum - 1, -1, -1):
            res.append(res[i] + halfNum)
        return res
