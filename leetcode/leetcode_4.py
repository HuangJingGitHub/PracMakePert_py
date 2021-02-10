# two-finger pointer method. It is tedious to set the correct indices, conditions and extreme cases anyway 
class Solution:
    def findMedianSortedArrays(self, nums1: List[int], nums2: List[int]) -> float:
        len1 = len(nums1)
        len2 = len(nums2)

        if len1 + len2 <= 1:
            if len1 == 1:
                return nums1[0]
            elif len2 == 1:
                return nums2[0] 
            else:
                return 0            
        targetCount = (len1 + len2) // 2
        count, idx1, idx2 = 1, 0, 0

        while count < targetCount:
            if idx2 >= len2 or (idx1 < len1 and nums1[idx1] <= nums2[idx2]):
                idx1 += 1
            else:
                idx2 += 1
            count += 1
        

        temp = []
        for i in range(2):
            if idx2 >= len2 or (idx1 < len1 and nums1[idx1] <= nums2[idx2]):
                temp.append(nums1[idx1])
                idx1 += 1
            else:
                temp.append(nums2[idx2])
                idx2 += 1            

        if (len1 + len2) % 2 == 0:
            return (temp[0] + temp[1]) / 2
        else:
            return temp[1]
