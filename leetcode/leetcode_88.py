class Solution:
    def merge(self, nums1: List[int], m: int, nums2: List[int], n: int) -> None:
        """
        Do not return anything, modify nums1 in-place instead.
        """
        if m == 0:
            for i in range(n):
                nums1[i] = nums2[i]
            return
        if n == 0:
            return
        
        resIdx = m + n - 1
        pt1 = m - 1
        pt2 = n - 1

        while resIdx >= 0:
            if nums1[pt1] >= nums2[pt2]:
                nums1[resIdx] = nums1[pt1]
                pt1 -= 1
            else:
                nums1[resIdx] = nums2[pt2]
                pt2 -= 1
            resIdx -= 1
            if pt1 == -1 or pt2 == -1:
                break

        if pt1 == -1:
            for j in range(pt2, -1, -1):
                nums1[resIdx] = nums2[j]
                resIdx -= 1
        
