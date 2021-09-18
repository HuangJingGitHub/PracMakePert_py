class Solution:
    def simplifyPath(self, path: str) -> str:
        strs = []
        splitStr = path.split('/')
        res = ""

        for temp in splitStr:
            if temp == "" or temp == '.':
                continue
            elif temp == '..' and strs:
                strs.pop(-1)
            elif temp != '..':
                strs.append(temp)
        for temp in strs:
            res += ('/' + temp)
        if not strs:
            return '/'
        return res
