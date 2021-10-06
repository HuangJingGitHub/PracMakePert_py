# Definition for singly-linked list.
# class ListNode:
#     def __init__(self, val=0, next=None):
#         self.val = val
#         self.next = next
class Solution:
    def deleteDuplicates(self, head: ListNode) -> ListNode:
        res = head
        while head:
            refVal = head.val
            tempHead = head
            while head and head.val == refVal:
                head = head.next
            tempHead.next = head
        return res
