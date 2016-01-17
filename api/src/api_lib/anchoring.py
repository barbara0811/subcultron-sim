#!/usr/bin/env python

"""
Anchoring API class.
"""

class Anchor(object):
    
    def __init__(self):
        self.status = -1
    
    def hold(self):
        '''
        Activate anchor.
        Returns:
            success: bool
        '''
        pass
    
    def release(self):
        '''
        Deactivate anchor.
        Returns:
            success: bool
        '''
        pass
    
    def check_status(self):
        '''
        Check anhor status.
        Returns:
            status: int, predefined status id
        '''
        
        return self.status