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

# log(len(nums1) + len(nums2)) time        
class Solution:
    def findMedianSortedArrays(self, nums1: List[int], nums2: List[int]) -> float:
        totalLen = len(nums1) + len(nums2)
        if len(nums1) > len(nums2):
            return self.findMedianSortedArrays(nums2, nums1)
        if len(nums1) == 0:
            if len(nums2) % 2 != 0:
                return nums2[len(nums2) // 2]
            else:
                return (nums2[len(nums2) // 2] + nums2[len(nums2) // 2 - 1]) / 2
            
        lEdge = 0 
        rEdge = len(nums1)
        curA = 0
        curB = 0
        res = 0.0

        while lEdge <= rEdge:
            curA = lEdge + (rEdge - lEdge) // 2
            curB = (totalLen + 1) // 2 - curA
            if curA == 0:
                LA = -100000000
            else:
                LA = nums1[curA - 1]
            if curA == len(nums1):
                RA = 100000000
            else:
                RA = nums1[curA]

            if curB == 0:
                LB = -100000000
            else:
                LB = nums2[curB - 1]
            if curB == len(nums2):
                RB = 100000000
            else:
                RB = nums2[curB]

            if LA > RB:
                rEdge = curA - 1
            elif LB > RA:
                lEdge = curA + 1
            else:
                if totalLen % 2 != 0:
                    result = max(LA, LB)
                else:
                    result = (max(LA, LB) + min(RA, RB)) / 2.0
                break
        return result           
                
        
