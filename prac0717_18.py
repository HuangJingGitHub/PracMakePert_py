import os
import time

def main():
    contents = 'How time flies !?'
    i = 1
    while i in range(100):
        os.system('cls')
        print(contents)
        time.sleep(1)
        contents = contents[1:] + contents[0]

if __name__ == '__main__':
    main()
