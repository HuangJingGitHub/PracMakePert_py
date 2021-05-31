class Solution:
    def insert(self, intervals: List[List[int]], newInterval: List[int]) -> List[List[int]]:
        res = []

        if len(intervals) == 0:
            return [newInterval]
        if newInterval[1] < intervals[0][0]:
            return [newInterval] + intervals
        if newInterval[0] > intervals[-1][1]:
            return intervals + [newInterval]

        start = 0
        for i in range(len(intervals)):
            if intervals[i][1] < newInterval[0]:
                res.append([intervals[i][0], intervals[i][1]])
            else:
                start = i
                break

        res.append([newInterval[0], newInterval[1]])

        for i in range(start, len(intervals)):
            if res[-1][1] < intervals[i][0]:
                res.append([intervals[i][0], intervals[i][1]])
            else:
                res[-1][0] = min(res[-1][0], intervals[i][0])
                res[-1][1] = max(res[-1][1], intervals[i][1])
        return res
