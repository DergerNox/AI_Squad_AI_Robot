"""agent.py: the AI driver for the PiCar.

Type a task here (or send one as text on /from_human) and the AI drives the
car with the tools in picar_tools.py. Answers are printed and sent on /to_human.

Which AI and which car are set in .env (see .env.example), no code changes needed:
    PICAR_LLM, PICAR_MODEL, PICAR_BASE_URL, PICAR_TEMPERATURE, the API keys: the model,
                    see llm.py
    PICAR_CONFIG    which car: sim (the twin, the default), car (the real PiCar-B) or a
                    settings file (see car_config.py)
"""

import logging
import os
import signal
import threading
import warnings

# Remove ROS 2 UNIX timestamps from terminal output
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{name}] [{severity}]: {message}"

try:   # read the settings and keys from .env next to this file
    from dotenv import load_dotenv
    load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))
except ImportError:   # without python-dotenv, export the settings in the shell instead
    pass

from langchain_core.callbacks import StreamingStdOutCallbackHandler
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_core.runnables import RunnableConfig
from langgraph.errors import GraphRecursionError
from rai.agents import BaseAgent
from rai.agents.langchain import HRICallbackHandler, ReActAgentState, create_react_runnable
from rai.communication.ros2 import ROS2Connector, ROS2Context, ROS2HRIConnector, ROS2HRIMessage
from rclpy.logging import get_logger

from car_config import load_car
from llm import make_llm
from picar_tools import make_picar_tools, save_memory
from prompt import picar_prompt

# Hide the log lines for every model request and every streamed word
logging.getLogger("httpx").setLevel(logging.WARNING)
logging.getLogger("rai.agents.langchain.callback").setLevel(logging.WARNING)
logging.getLogger("ROS2Connector").setLevel(logging.ERROR)      # "executor is behind" notices
logging.getLogger("ROS2HRIConnector").setLevel(logging.ERROR)
warnings.filterwarnings("ignore", message="Calling .text")   # harmless RAI deprecation notice

# Sent when the model stops without finishing (small models often announce a step and stop)
NUDGE = ("You stopped before finishing. Continue now and call a tool, and look before you "
         "say what you see. If the task is finished, or cannot be done, answer starting with DONE.")


class PicarAgent(BaseAgent):
    def __init__(self):
        super().__init__()
        self.hri_connector = ROS2HRIConnector()   # text in and out
        self.ros2_connector = ROS2Connector()     # the car's services and camera
        llm, description = make_llm()
        get_logger("agent").info(f"AI model: {description}")
        car = load_car()   # the car's numbers, from PICAR_CONFIG's settings file
        get_logger("agent").info(f"Car: {car.description} ({car.file})")
        self.prompt = picar_prompt(car)
        # The same model also answers the tools' short questions about pictures
        tools, self.new_task, self.memory = make_picar_tools(self.ros2_connector, llm, car)
        self.agent = create_react_runnable(
            llm=llm,
            tools=tools,
            system_prompt=self.prompt,
        )
        self.state = ReActAgentState(messages=[])
        self.callbacks = [
            HRICallbackHandler(connectors={"/to_human": self.hri_connector}),
            StreamingStdOutCallbackHandler(),   # also print the answer here
        ]
        self.busy = threading.Lock()   # one task at a time; a new one waits for the last
        self.hri_connector.register_callback("/from_human", self.input_callback)

    def input_callback(self, message: ROS2HRIMessage):
        text = message.text.strip() if message.text else ""
        if text:
            self.run_task(text)

    def run_task(self, text):
        with self.busy:
            get_logger("agent").info(f"USER: {text}")
            # Each task starts with a fresh conversation, because the small model copies
            # its old answers, but it gets the facts it saved with remember.
            notes = "\n".join(f"- {fact}" for fact in self.memory) or "- nothing yet"
            self.state = ReActAgentState(messages=[
                SystemMessage(content=f"{self.prompt}\nWhat you remember from earlier tasks:\n{notes}"),
                HumanMessage(content=text),
            ])
            self.new_task()   # forget the targets reached in the last task
            # Each tool call takes two steps; LangGraph's default limit of 25 is too few.
            config = RunnableConfig(callbacks=self.callbacks, recursion_limit=100)
            try:
                for attempt in range(5):   # nudge it on until it has used a tool and says DONE
                    if attempt > 0:
                        print()
                        self.nudge()
                    try:
                        # stream instead of invoke: self.state follows every step, so
                        # hitting the step limit keeps the conversation and the progress
                        for state in self.agent.stream(self.state, config=config, stream_mode="values"):
                            self.state = state
                    except GraphRecursionError:
                        get_logger("agent").warning("Step limit reached, nudging it on.")
                        continue
                    used_tools = any(isinstance(m, ToolMessage) for m in self.state["messages"])
                    if "DONE" in str(self.state["messages"][-1].content) and used_tools:
                        break
            except Exception as error:
                # A half-finished step would break the next request, so start over.
                get_logger("agent").error(f"Task stopped: {error}")
                get_logger("agent").error("Starting a new conversation.")
                self.state = ReActAgentState(messages=[])
            print()

    def nudge(self):
        messages = self.state["messages"]
        # A tool call that the step limit cut off has no result: drop it
        if messages and isinstance(messages[-1], AIMessage) and messages[-1].tool_calls:
            messages.pop()
        messages.append(HumanMessage(content=NUDGE, name="nudge"))

    def run(self):
        pass

    def stop(self):
        self.ros2_connector.shutdown()
        self.hri_connector.shutdown()


@ROS2Context()
def main():
    agent = PicarAgent()
    # Let Ctrl+C stop input() at once (ROS's own handler waits for Enter)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    print("PiCar agent ready. Type a task and press Enter, 'forget' to clear its memory, Ctrl+C to quit.")
    try:
        while True:
            text = input("> ").strip()
            if text == "forget":   # clear what the AI remembers
                agent.memory.clear()
                save_memory(agent.memory)
                print("Memory cleared.")
            elif text:
                agent.run_task(text)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        agent.stop()


if __name__ == "__main__":
    main()