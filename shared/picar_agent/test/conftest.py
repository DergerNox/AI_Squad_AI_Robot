"""Setup for the agent's tests: lets them import picar_tools from the folder above.

The tests need the agent's Python packages (RAI, LangChain, rclpy), but no running
ROS, Gazebo or AI model: a fake car, a fake vision model and a fake clock stand in.

    source /opt/ros/jazzy/setup.bash
    source ~/rai_venv/bin/activate
    cd picar_agent && python3 -m pytest test
"""

import os
import sys

AGENT_FOLDER = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, AGENT_FOLDER)

try:
    import picar_tools  # noqa: F401  (only to check that it can be imported)
except ImportError as error:
    raise ImportError(f"{error}. Source ROS and activate ~/rai_venv first "
                      "(see the top of test/conftest.py).") from error
