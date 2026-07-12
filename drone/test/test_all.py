"""
test_all.py — 综合测试入口
运行所有模块的测试
"""

import sys
import os
import unittest

# Ensure drone package is in path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import all test modules
from test_lx_protocol import *
from test_path_plan import *
from test_localization import *
from test_mcu_serial import *
from test_state_machine import *
from test_laser_led import *
from test_h7_gpio_protocol import *


if __name__ == '__main__':
    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # 发现所有测试
    suite.addTests(loader.loadTestsFromModule(sys.modules[__name__]))

    # 运行
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # 输出汇总
    print("\n" + "=" * 60)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    if result.wasSuccessful():
        print("✅ ALL TESTS PASSED")
    else:
        print("❌ SOME TESTS FAILED")
        for test, traceback in result.failures + result.errors:
            print(f"\n  FAIL: {test}")
            print(f"  {traceback.split(chr(10))[-2]}")
