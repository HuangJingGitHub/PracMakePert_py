class Solution:
    def minRemoveToMakeValid(self, s: str) -> str:
        idx = []

        for i, c in enumerate(s):
            if c == '(':
                idx.append(i)
            elif c == ')':
                if idx and s[idx[-1]] == '(':
                    idx.pop();
                else:
                    idx.append(i)
        
        while idx:  # pay attention traverse from the back as s is dynamically change. Otherwise the idx need to add a shift
            pos = idx.pop()
            s = s[: pos] + s[pos+1 :]
            
        return s 
