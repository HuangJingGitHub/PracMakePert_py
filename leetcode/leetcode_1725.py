class Solution:
    def countGoodRectangles(self, rectangles: List[List[int]]) -> int:
        res = 1
        maxLen  = min(rectangles[0])
        
        for i in range(1, len(rectangles)):
            curLen = min(rectangles[i])
            if curLen == maxLen:
                res += 1
            elif curLen > maxLen:
                maxLen = curLen
                res = 1
        return res
