"""Estimate client minus ROS clock offset using bounded four-timestamp exchanges."""
from collections import deque
import math


class ClockSync:
    def __init__(self, max_rtt=.1, ttl=5., min_samples=3):
        self.max_rtt=max_rtt;self.ttl=ttl;self.min_samples=min_samples
        self.samples=deque(maxlen=20)
        self.session=None

    def reset(self):
        self.samples.clear()

    def estimate(self, now):
        while self.samples and now-self.samples[0][0]>self.ttl:
            self.samples.popleft()
        if len(self.samples)<self.min_samples: return None
        _,rtt,offset=min(self.samples,key=lambda sample:sample[1])
        return offset,rtt

    def add(self,t0,t1,t2,t3,elapsed,now,session):
        """All timestamps integer nanoseconds; elapsed is host monotonic RTT."""
        if not all(type(t) is int and t>0 for t in (t0,t1,t2,t3)):
            raise ValueError('invalid clock timestamps')
        if not isinstance(session,str) or not session or len(session)>64:
            raise ValueError('invalid client session')
        if not math.isfinite(elapsed) or elapsed<0 or elapsed>self.max_rtt:
            raise ValueError('clock round trip too slow')
        if abs((t3-t0)/1e9-elapsed)>.02:
            self.reset();raise ValueError('ROS clock changed during exchange')
        processing=(t2-t1)/1e9
        rtt=elapsed-processing
        if processing<0 or rtt<-.002 or rtt>self.max_rtt:
            raise ValueError('invalid client processing time')
        offset=((t1-t0)+(t2-t3))//2
        changed=self.session is not None and self.session!=session
        estimate=self.estimate(now)
        if changed or (estimate is not None and abs(offset-estimate[0])>50_000_000):
            self.reset();changed=True
        self.session=session
        self.samples.append((now,max(0.,rtt),offset))
        return changed
