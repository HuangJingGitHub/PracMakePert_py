# Definition for singly-linked list.
# class ListNode:
#     def __init__(self, val=0, next=None):
#         self.val = val
#         self.next = next
class Solution:
    def deleteDuplicates(self, head: ListNode) -> ListNode:
        if head == None:
            return head
            
        newNode = None
        res = None
        while head:
            refVal = head.val
            refNode = head
            cnt = 0
            while head and head.val == refVal:
                head = head.next
                cnt += 1
            if cnt == 1 and res == None:
                newNode = refNode
                res = newNode
            elif cnt == 1 and res != None:
                newNode.next = refNode
                newNode = newNode.next
        if res:
            newNode.next = None
        return res
