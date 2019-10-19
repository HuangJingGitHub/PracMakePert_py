class solutionLeetcode_7:
    def reverse(self, x: int) -> int:
        s = str(x)[::-1].strip('-')
        if int(s) < 2**31:
            if x >= 0:
                return int(s)
            else:
                return 0 - int(s)
        return 0

class solutionLeetcode_1:
    def twoSum(self, nums, target):
        hashmap = {}
        for index, num in enumerate(nums):
            another_num = target - num
            if another_num in hashmap:
                return [hashmap[another_num], index]
            hashmap[num] = index
        return None
    
test_x = int(110)
soln = solutionLeetcode_7()
reverse_x = soln.reverse(soln, test_x)

print(reverse_x)
