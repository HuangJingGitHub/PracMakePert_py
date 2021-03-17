class Solution:
    def findSubstring(self, s: str, words: List[str]) -> List[int]:
        wordFreq = collections.Counter(words)
        wordNum = len(words)
        wordLen = len(words[0])
        res = []

        i = 0
        while i <= len(s) - wordNum * wordLen:
            if wordFreq[s[i : i + wordLen]] == 0:
                i += 1
                continue
            j = i
            wordLog = collections.defaultdict(int)
            while j < i + wordNum * wordLen:
                curWord = s[j : j + wordLen]
                wordLog[curWord] += 1
                if wordFreq[curWord] == 0 or wordLog[curWord] > wordFreq[curWord]:
                    break
                j += wordLen
            if j == i + wordNum * wordLen:
                res.append(i)
            i += 1
        return res
