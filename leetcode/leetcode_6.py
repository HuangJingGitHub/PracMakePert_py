class Solution:
    def convert(self, s: str, numRows: int) -> str:
        if numRows < 2:
            return s
        rowStr = ["" for _ in range(numRows)]
        goDown = False
        rowIdx = 0

        for c in s:
            rowStr[rowIdx] += c
            if rowIdx == 0 or rowIdx == numRows - 1:
                goDown = not goDown
            if goDown:
                rowIdx += 1
            else:
                rowIdx -= 1
        return "".join(rowStr)
