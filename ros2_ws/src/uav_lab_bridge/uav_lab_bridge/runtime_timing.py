"""Bounded observation of source/receipt timing; never an authority or gate."""
from collections import OrderedDict
import math
import threading


class RuntimeTiming:
    def __init__(self):
        self.channels=OrderedDict()
        self.lock=threading.Lock()

    def arrival(self,name,source_s,now):
        if not all(math.isfinite(v) for v in (source_s,now)):
            raise ValueError('finite source and monotonic receipt times required')
        with self.lock:
            previous=self.channels.pop(name,None)
            self.channels[name]={'source_s':source_s,'received_at':now,
                'count':1 if previous is None else previous['count']+1,
                'source_advance_s':None if previous is None else source_s-previous['source_s'],
                'maximum_receipt_gap_s':0. if previous is None else max(
                    previous['maximum_receipt_gap_s'],now-previous['received_at'])}
            while len(self.channels)>16:self.channels.popitem(last=False)

    def snapshot(self,now,clock_s):
        with self.lock:
            return {name:{**item,'receipt_age_s':now-item['received_at'],
                'source_age_s':clock_s-item['source_s']} for name,item in self.channels.items()}
