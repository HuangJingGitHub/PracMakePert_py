class Solution:
    def merge(self, nums1: List[int], m: int, nums2: List[int], n: int) -> None:
        """
        Do not return anything, modify nums1 in-place instead.
        """
        pt1 = 0
        pt2 = 0

        for i in range(m + n):
            if nums1[pt1] <= nums2[pt2]:
                pt1 += 1
            else:
                for j in range(pt)
