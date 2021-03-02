class Solution:
    def isValid(self, s: str) -> bool:
        stackChar = []

        for ch in s:
            if (not stackChar) or (ch in ['(', '[', '{']):
                stackChar.append(ch)
            elif ch == ')':
                if stackChar[-1] == '(':
                    stackChar.pop()
                else:
                    stackChar.append(ch)
            elif ch == ']':
                if stackChar[-1] == '[':
                    stackChar.pop()
                else:
                    stackChar.append(ch)
            else:
                if stackChar[-1] == '{':
                    stackChar.pop()
                else:
                    stackChar.append(ch)
        return not stackChar
