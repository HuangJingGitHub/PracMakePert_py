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

            # interval = (k - 1) // n! instead of k // n! in order to deal with cases where k % n! == 0, 
            # in these cases, interval index is k // n! - 1. If k cannot be divided by n!, interval = k // n! = (k - 1) // n!
            # so following code can also be used
            # if k % math.factorial(n) == 0:
            #     interval = k // math.factorial(n) - 1
            # else:
            #     interval = k // math.factorial(n)
