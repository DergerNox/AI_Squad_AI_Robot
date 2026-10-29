"""agent.py: the AI driver for the PiCar.

Type a task here (or send one as text on /from_human) and the AI drives the
car with the tools in picar_tools.py. Answers are printed and sent on /to_human.

Which AI it uses is set in .env (see .env.example), no code changes needed:
    PICAR_LLM       ollama (local, the default), openai (OpenAI or any server with an
                    OpenAI-compatible API) or google (Gemini)
    PICAR_MODEL     the model name; it must understand pictures and call tools
    PICAR_BASE_URL  the server address, if it is not the default one
    OPENAI_API_KEY / GOOGLE_API_KEY   the key for a cloud model
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
from langchain_ollama import ChatOllama
from langgraph.errors import GraphRecursionError
from rai.agents import BaseAgent
from rai.agents.langchain import HRICallbackHandler, ReActAgentState, create_react_runnable
from rai.communication.ros2 import ROS2Connector, ROS2Context, ROS2HRIConnector, ROS2HRIMessage
from rclpy.logging import get_logger

from picar_tools import PICAR_PROMPT, compress_history, make_picar_tools, save_memory

# Hide the log lines for every model request and every streamed word
logging.getLogger("httpx").setLevel(logging.WARNING)
logging.getLogger("rai.agents.langchain.callback").setLevel(logging.WARNING)
logging.getLogger("ROS2Connector").setLevel(logging.ERROR)      # "executor is behind" notices
logging.getLogger("ROS2HRIConnector").setLevel(logging.ERROR)
warnings.filterwarnings("ignore", message="Calling .text")   # harmless RAI deprecation notice

# Sent when the model stops without finishing (small models often announce a step and stop)
NUDGE = ("You stopped before finishing. Continue now and call a tool, and look before you "
         "say what you see. If the task is finished, or cannot be done, answer starting with DONE.")


class CompactHistory:
    """Mixed into a chat model class: the model gets a compressed copy of the conversation."""

    # run_manager must be named here: LangChain only streams the words to the
    # callbacks when the method's signature has it.
    def _generate(self, messages, stop=None, run_manager=None, **kwargs):
        return super()._generate(compress_history(messages), stop, run_manager, **kwargs)

    def _stream(self, messages, stop=None, run_manager=None, **kwargs):
        return super()._stream(compress_history(messages), stop, run_manager, **kwargs)


class CompactChatOllama(CompactHistory, ChatOllama):
    pass


def make_llm():
    """The chat model chosen in .env. Returns the model and a short description."""
    provider = os.environ.get("PICAR_LLM", "ollama").strip().lower()
    model = os.environ.get("PICAR_MODEL", "").strip()
    base_url = os.environ.get("PICAR_BASE_URL", "").strip() or None
    if provider not in ("ollama", "openai", "google"):
        raise ValueError(f"PICAR_LLM must be ollama, openai or google, not '{provider}'")
    if provider != "ollama" and not model:
        raise ValueError(f"Set PICAR_MODEL in .env: the {provider} model to use")

    if provider == "ollama":
        model = model or "qwen3-vl:4b-instruct"
        # compress_history keeps each request well under num_ctx
        llm = CompactChatOllama(model=model, base_url=base_url, num_ctx=8192, temperature=0.2)
    elif provider == "openai":   # the key comes from OPENAI_API_KEY
        from langchain_openai import ChatOpenAI

        class CompactChatOpenAI(CompactHistory, ChatOpenAI):
            pass

        llm = CompactChatOpenAI(model=model, base_url=base_url, temperature=0.2)
    elif provider == "google":   # the key comes from GOOGLE_API_KEY
        from langchain_google_genai import ChatGoogleGenerativeAI

        class CompactGemini(CompactHistory, ChatGoogleGenerativeAI):
            pass

        llm = CompactGemini(model=model, temperature=0.2)
    return llm, f"{provider}, {model}" + (f" at {base_url}" if base_url else "")


class PicarAgent(BaseAgent):
    def __init__(self):
        super().__init__()
        self.hri_connector = ROS2HRIConnector()   # text in and out
        self.ros2_connector = ROS2Connector()     # the car's services and camera
        llm, description = make_llm()
        get_logger("agent").info(f"AI model: {description}")
        # The same model also answers the tools' short questions about pictures
        tools, self.new_task, self.memory = make_picar_tools(self.ros2_connector, llm)
        self.agent = create_react_runnable(
            llm=llm,
            tools=tools,
            system_prompt=PICAR_PROMPT,
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
                SystemMessage(content=f"{PICAR_PROMPT}\nWhat you remember from earlier tasks:\n{notes}"),
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