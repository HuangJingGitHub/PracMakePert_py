class Solution:
    def generateParenthesis(self, n: int) -> List[str]:
        if n == 0:
            return [""]
        elif n == 1:
            return ["()"]
        
        dp = []
        dp.append([""])
        dp.append(["()"])

        for i in range(2, n + 1):
            curList = []
            for j in range(0, len(dp)):
                for str1 in dp[j]:
                    for str2 in dp[len(dp) - 1 - j]:
                        curList.append('(' + str1 + ')' + str2)
            dp.append(curList)
        return dp[-1]
