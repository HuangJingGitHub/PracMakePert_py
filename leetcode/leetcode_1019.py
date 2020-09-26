# Definition for singly-linked list.
# class ListNode:
#     def __init__(self, x):
#         self.val = x
#         self.next = None

class Solution:
    def nextLargerNodes(self, head: ListNode) -> List[int]:
        if head == None:
            return []
        
        valList = []
        while head:
            valList.append(head.val)
            head = head.next
        
        stack = []  # Inbuilt implementation of stack in python is list. pop(), append() are O(1) operations.
        stack.append(valList[-1])
        valList[-1] = 0
        
        for i in range(len(valList)-2, -1, -1 ):
            temp = valList[i]

            while stack and stack[-1] <= temp:
                stack.pop()
            if stack:
                valList[i] = stack[-1]
            else:
                valList[i] = 0
            stack.append(temp)
        
        return valList
