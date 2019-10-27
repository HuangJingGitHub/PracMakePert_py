class solutionLeetcode_3:
    def lengthOfLongestSubstring(self, s: str) -> (int, str):
        if not s:
            return 0
        left = 0
        lookup = set()
        n = len(s)
        max_len = 0
        cur_len = 0
        for i in range(n):
            cur_len += 1
            while s[i] in lookup:
                lookup.remove(s[left])
                left += 1
                cur_len -= 1
            if cur_len > max_len:
                max_len = cur_len
            lookup.add(s[i])

        longestSubstring = s[left:left+max_len+1]
        return max_len, longestSubstring


class solutionLeetcode_4:
    def findMedianSortedArrays(self, nums1, nums2):
        nums1.extend(nums2)
        nums1.sort()
        if len(nums1) % 2 == 0:
            return sum(nums1[[len(nums1)//2]-1:len(nums1)//2+1])/2
        else:
            return nums1[(len(nums1)-1)//2]

class solutionLeetcode_7:
    def reverse(self, x) -> int:
        s = str(x)[::-1].strip('-')
        if int(s) < 2**31:
            if x >= 0:
                return int(s)
            else:
                return 0 - int(s)
        return 0

test_x = int(110)
soln = solutionLeetcode_7()
reverse_x = soln.reverse(test_x)
print(reverse_x)

testStrQ3 = 'fsafesvafdsag'
solnQ3 = solutionLeetcode_3()
_, longestSubstring = solnQ3.lengthOfLongestSubstring(testStrQ3)
print(longestSubstring)