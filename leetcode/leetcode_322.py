class Solution:
    def coinChange(self, coins: List[int], amount: int) -> int:
        if amount == 0:
            return 0
        
        dp = [1e8 for i in range(amount + 1)]
        dp[0] = 0

        for i in range(1, amount + 1):
            for coin in coins:
                if i >= coin and dp[i - coin] != -10:
                    dp[i] = min(dp[i], dp[i - coin] + 1) 
        
        if dp[-1] == 1e8:
            return -1
        return dp[-1]
