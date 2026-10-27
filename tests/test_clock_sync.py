import unittest
from teleop_core.clock_sync import ClockSync


class ClockSyncTests(unittest.TestCase):
    def setUp(self):
        self.sync=ClockSync()
        self.epoch=1_790_000_000_000_000_000

    def sample(self,now=1.,offset=.113,forward=.01,backward=.01,session='quest',processing=.001):
        t0=self.epoch+int(now*1e9)
        t1=t0+int((forward+offset)*1e9)
        t2=t1+int(processing*1e9)
        elapsed=forward+processing+backward
        t3=t0+int(elapsed*1e9)
        return self.sync.add(t0,t1,t2,t3,elapsed,now,session)

    def test_positive_and_negative_offsets(self):
        for offset in (.113,-.25):
            self.sync=ClockSync()
            for now in (1.,1.5,2.): self.sample(now,offset)
            ns,rtt=self.sync.estimate(2.)
            self.assertAlmostEqual(ns/1e9,offset,places=8)
            self.assertAlmostEqual(rtt,.02)
            # A future-dated client pose becomes its actual delivery age.
            raw_age=.012-offset
            self.assertAlmostEqual(raw_age+ns/1e9,.012)

    def test_requires_three_samples_and_expires(self):
        self.sample();self.sample(1.5)
        self.assertIsNone(self.sync.estimate(1.5))
        self.sample(2.)
        self.assertIsNotNone(self.sync.estimate(2.))
        self.assertIsNone(self.sync.estimate(7.1))

    def test_selects_lowest_rtt(self):
        self.sample(1.,forward=.03,backward=.01)
        self.sample(1.5,forward=.01,backward=.01)
        self.sample(2.,forward=.04,backward=.01)
        self.assertAlmostEqual(self.sync.estimate(2.)[0]/1e9,.113)

    def test_slow_exchange_rejected(self):
        with self.assertRaises(ValueError): self.sample(forward=.2)
        self.assertIsNone(self.sync.estimate(1.))

    def test_session_restart_requires_new_bootstrap(self):
        for now in (1.,1.5,2.): self.sample(now)
        self.assertTrue(self.sample(2.5,session='restarted'))
        self.assertIsNone(self.sync.estimate(2.5))

    def test_clock_jump_invalidates_estimate(self):
        for now in (1.,1.5,2.): self.sample(now)
        self.assertTrue(self.sample(2.5,offset=.313))
        self.assertIsNone(self.sync.estimate(2.5))

    def test_host_clock_jump_and_malformed_samples(self):
        for now in (1.,1.5,2.): self.sample(now)
        with self.assertRaises(ValueError):
            self.sync.add(self.epoch,self.epoch,self.epoch,self.epoch+1_000_000_000,.02,3.,'quest')
        self.assertIsNone(self.sync.estimate(3.))
        for bad in (0,True,1.,None):
            with self.assertRaises(ValueError):
                self.sync.add(bad,self.epoch,self.epoch,self.epoch,.02,3.,'quest')


if __name__=='__main__': unittest.main()
