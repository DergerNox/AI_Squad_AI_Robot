"""The full agent: one task with two targets, the stand-in model follows the tools' instructions."""
import os, sys, time
sys.path.insert(0, os.path.abspath(sys.argv[1])); os.chdir(os.path.abspath(sys.argv[1]))
import agent, rclpy
rclpy.init()
a = agent.PicarAgent()
time.sleep(2)
start = time.time()
a.run_task(sys.argv[2])
print(f"\nTASK TIME {time.time() - start:.1f} s")
a.stop(); rclpy.shutdown()
