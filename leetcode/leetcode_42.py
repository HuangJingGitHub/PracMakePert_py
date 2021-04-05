class Solution:
    def trap(self, height: List[int]) -> int:   
        if len(height) < 3:
            return 0
        
        max_left = [0] * len(height)
        max_right = [0] * len(height)

        for i in range(1, len(height)):
            max_left[i] = max(height[i - 1], max_left[i - 1])
        for i in range(len(height) - 2, 0, -1):
            max_right[i] = max(height[i + 1], max_right[i + 1])
        
        res = 0
        for i in range(1, len(height) - 1):
            min_height = min(max_left[i], max_right[i])
            if min_height > height[i]:
                res += min_height - height[i]
        return res
