# Definition for singly-linked list.
# class ListNode:
#     def __init__(self, val=0, next=None):
#         self.val = val
#         self.next = next
class Solution:
    def reverseKGroup(self, head: ListNode, k: int) -> ListNode:
        dummy = ListNode(0, head)
        cur = head
        listLen = 0
        while cur:
            cur = cur.next
            listLen += 1
        
        groupPre = dummy
        while listLen >= k:
            cur = groupPre.next
            pre = None
            for i in range(0, k):
                nextNode = cur.next
                cur.next = pre
                pre = cur
                cur = nextNode
            temp = groupPre.next
            groupPre.next.next = cur
            groupPre.next = pre
            groupPre = temp
            listLen -= k 
        return dummy.next
