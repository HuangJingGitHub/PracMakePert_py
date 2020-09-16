# Definition for singly-linked list.
# class ListNode:
#     def __init__(self, x):
#         self.val = x
#         self.next = None

class Solution:
    def addTwoNumbers(self, l1: ListNode, l2: ListNode) -> ListNode:
        sumDigit = 0;
        headNode = ListNode(-1)
        res = headNode  # shallow copy, res is just another name of instantaneous headNode
        while l1 != None or l2 != None or sumDigit > 0:
            if l1 != None:
                sumDigit += l1.val
                l1 = l1.next
            
            if l2 != None:
                sumDigit += l2.val
                l2 = l2.next

            newNode = ListNode(sumDigit % 10)
            sumDigit //= 10
            headNode.next = newNode
            headNode = headNode.next
        
        return res.next
