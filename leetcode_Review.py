class solutionLeetcode_7:
    def reverse(self, x: int) -> int:
        s = str(x)[::-1].strip('-')
        if int(s) < 2**31:
            if x >= 0:
                return int(s)
            else:
                return 0 - int(s)
        return 0

test_x = int(110)
soln = solutionLeetcode_7()
reverse_x = soln.reverse(soln, test_x)

print(reverse_x)
