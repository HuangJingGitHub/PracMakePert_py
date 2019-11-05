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


class solutionLeetcode_5:
    def longestPalindrome(self, s):
        str_length = len(s)
        max_length = 0
        start = 0
        for i in range(str_length):
            if i - max_length >= 1 and s[i - max_length - 1:i + 2] == s[i - max_length - 1:i+2][::-1]:
                start = i - max_length - 1
                max_length += 2
                continue
            if i - max_length >= 0 and s[i - max_length:i + 2] == s[i - max_length:i + 2][::-1]:
                start = i - max_length
                max_length += 1
        return s[start:start + max_length + 1]

class solutionLeetcode_6:
    def convert(self, s:str, numRows:int) -> list:
        if numRows < 2:
            return s
        res = ["" for _ in range(numRows)]  # "" stands for string
        i, flag = 0, -1
        for c in s:
            res[i] += c
            if i == 0 or i == numRows - 1:
                flag = -flag
            i += flag
        return "".join(res)

class solutionLeetcode_7:
    def reverse(self, x) -> int:
        s = str(x)[::-1].strip('-')   # use extended slices to reverse a string.
        if int(s) < 2**31:
            if x >= 0:
                return int(s)
            else:
                return 0 - int(s)
        return 0


class solutionLeetcode_8:
        def myAtoi(self, str: str) -> int:
            validChar = ['-', '+', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
            validNumber = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
            if len(str) == 0:
                return 0
            startIndex = -1
            endIndex = 0
            initialChar = True
            lastEntryValid = True
            for i in range(len(str)):
                if str[i] == ' ':
                    continue
                if (str[i] not in validChar):
                    return 0
                if (str[i] in validChar) and initialChar:
                    initialChar = False
                    startIndex = i
                    if i == len(str) - 1:
                        if str[i] in ['-', '+']:
                            return 0
                        else:
                            endIndex = len(str)
                            break
                    continue
                if (str[i] not in validNumber) and (not initialChar):
                    endIndex = i
                    lastEntryValid = False

            if startIndex == -1:
                return 0
            if lastEntryValid:
                endIndex = len(str)

            numberStr = str[startIndex:endIndex]
            return int(numberStr)

test_x = int(110)
soln = solutionLeetcode_7()
reverse_x = soln.reverse(test_x)
print(reverse_x)

testStrQ3 = 'fsafesvafdsag'
solnQ3 = solutionLeetcode_3()
_, longestSubstring = solnQ3.lengthOfLongestSubstring(testStrQ3)
print(longestSubstring)