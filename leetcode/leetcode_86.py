# Definition for singly-linked list.
# class ListNode:
#     def __init__(self, val=0, next=None):
#         self.val = val
#         self.next = next
class Solution:
    def partition(self, head: ListNode, x: int) -> ListNode:
        leftHead = ListNode()
        leftPt = leftHead
        rightHead = ListNode()
        rightPt = rightHead

        while head:
            if head.val < x:
                leftPt.next = head
                leftPt = leftPt.next
            else:
                rightPt.next = head
                rightPt = rightPt.next
            head = head.next
        
        leftHead = leftHead.next
        rightHead = rightHead.next
        
        rightPt.next = None
        if leftHead:
            leftPt.next = rightHead
            return leftHead
        return rightHead
