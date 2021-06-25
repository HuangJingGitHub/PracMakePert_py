class Solution:
    def hIndex(self, citations: List[int]) -> int:
        paperNum = len(citations)
        left = 0
        right = paperNum - 1
        res = 0

        while left <= right:
            mid = left + (right - left) // 2
            tempHIndex = paperNum - mid
            if citations[mid] >= tempHIndex:
                res = temp
                right = mid - 1
            else:
                left = mid + 1
        
        return res
