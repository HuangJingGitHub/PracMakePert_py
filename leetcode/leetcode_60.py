class Solution:
    def getPermutation(self, n: int, k: int) -> str:
        nums = []
        res = ""
        
        for i in range(1, n + 1):
            nums.append(str(i))
        
        for i in range(0, n):
            n -= 1
            interval = (k - 1) // math.factorial(n)
            res += nums[interval]
            nums.pop(interval)
            k -= interval * math.factorial(n)
        
        return res
