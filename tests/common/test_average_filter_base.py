# tests/common/test_average_filter_base.py

import unittest
from core.AverageFilter import AverageFilter


class TestAverageFilterBase:

    class TestAverageFilterBase(unittest.TestCase):
        TIME_ADVANCE_STEP = 0.1

        def setUp(self):
            self.time_func = self.get_time_func()
            self.filter = AverageFilter(
                name="TestFilter", max_age=0.15, time_func=self.time_func
            )
            self.reset_time()

        def get_time_func(self):
            raise NotImplementedError

        def reset_time(self):
            raise NotImplementedError

        def advance_time(self, delta):
            raise NotImplementedError

        def test_add_and_retrieve_average(self):
            self.filter.add_value(10.0)
            avg = self.filter.get_average()
            self.assertEqual(avg, 10.0, "Average of single value should be the value itself")

            self.advance_time(0.1)
            self.filter.add_value(20.0)
            avg = self.filter.get_average()
            self.assertEqual(avg, 15.0, "Average of two values should be their mean")

        def test_value_expiry(self):
            self.filter.add_value(10.0)
            self.advance_time(0.1)
            self.filter.add_value(20.0)
            self.advance_time(0.1)
            self.filter.add_value(30.0)
            
            # The first value should have expired by now
            avg = self.filter.get_average()
            self.assertEqual(avg, 25.0, "Average should only include non-expired values")

        def test_empty_filter(self):
            avg = self.filter.get_average()
            self.assertIsNone(avg, "Average of empty filter should be None")

        def test_filter_empty_property(self):
            self.assertTrue(self.filter.is_empty, "Filter should be empty initially")
            self.filter.add_value(10.0)
            self.assertFalse(
                self.filter.is_empty,
                "Filter should not be empty after adding a value",
            )
            # Advance time to expire the value
            self.advance_time(0.2)
            self.filter.get_average()  # This should clean up expired values
            self.assertTrue(
                self.filter.is_empty, "Filter should be empty after value expires"
            )