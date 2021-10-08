class Solution:
    def largestRectangleArea(self, heights: List[int]) -> int:
        if len(heights) == 1:
            return heights[0]
        
        stk = []
        res = 0

        for i in range(0, len(heights)):
            while stk and heights[stk[-1]] > heights[i]:
                h = heights[stk[-1]]
                width = i
                stk.pop(-1)
                if stk:
                    width = i - stk[-1] - 1
                res = max(res, h * width)
            stk.append(i)

        while stk:
            h = heights[stk[-1]]
            width = len(heights)
            stk.pop(-1)
            if stk:
                width = len(heights) - stk[-1] - 1
            res = max(res, h * width)
        return res
