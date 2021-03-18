class Solution:
    def uncommonFromSentences(self, A: str, B: str) -> List[str]:
        res = []
        wordList = A.split() + B.split()
        wordDict = collections.Counter(wordList)
        for word in wordDict:
            if wordDict[word] == 1:
                res.append(word)
        return res
 

class Solution:
    def uncommonFromSentences(self, A: str, B: str) -> List[str]:
        wordList = A.split() + B.split()
        return [i for i in wordList if wordList.count(i) == 1]     
