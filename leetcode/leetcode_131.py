class Solution:
    def partition(self, s: str) -> List[List[str]]:
        lenS = len(s)
        res = list()
        if lenS == 0:
            res
        
        path = list()
        self.backTracking(s, 0, lenS, path, res)
        return res
    
    def backTracking(self, s: str, start: int, length: int, path: List[str], res: List[List[str]]) -> None:
        if start == length:
            res.append(path.copy())  # append a copy, not a object reference, otherwise it will change dynamically with path and be an empty list at last.
            return
        
        for i in range(start, length):
            if not self.isPalindrome(s, start, i):
                continue
            path.append(s[start : i + 1])
            self.backTracking(s, i + 1, length, path, res)
            path.pop()
        
    def isPalindrome(self, s: str, left: int, right: int) -> bool:
        while left < right:
            if not s[left] == s[right]:
                return False
            left += 1
            right -= 1
        return True
