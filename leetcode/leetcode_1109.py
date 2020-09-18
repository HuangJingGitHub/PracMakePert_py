class Solution:
    def corpFlightBookings(self, bookings: List[List[int]], n: int) -> List[int]:
        res = [0] * n
        
        for bk in bookings:
            res[bk[0] - 1] += bk[2]
            if bk[1] < n:
                res[bk[1]] -= bk[2]
        
        for i in range(1, n):
            res[i] += res[i-1]
        
        return res
