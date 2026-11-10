"""history.py: the conversation, made short enough for the model's context.

agent.py's CompactHistory sends every request through compress_history: finished tasks keep
only the request and the answer, the task in progress its last steps and its newest
picture, and that picture gets its tool's result as a caption.
"""

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from rai.messages import HumanMultimodalMessage


def is_request(message):
    """True for a task from the user (not a picture, not the agent's nudge)."""
    return type(message) is HumanMessage and message.name != "nudge"


def caption_picture(messages):
    """Write the newest picture's tool result next to the picture itself.

    RAI sends each picture as a separate message after the tool result. The model
    reads that message last and pays the most attention to it, so the notes
    (stuck, reached, next target) go there instead of "Image returned by a tool call".
    """
    if not messages or not isinstance(messages[-1], HumanMultimodalMessage):
        return   # the newest message is not a picture: the model reads that one last anyway
    i = len(messages) - 1
    picture = messages[i]
    results = [m for m in messages[:i] if isinstance(m, ToolMessage)]
    same_call = [m for m in results if m.tool_call_id == getattr(picture, "tool_call_id", None)]
    result = (same_call or results or [None])[-1]
    if result is None:
        return
    text = result.content
    if isinstance(text, list):   # RAI stores it as a list of parts: [{"type": "text", "text": ...}]
        text = " ".join(part.get("text", "") for part in text if isinstance(part, dict))
    messages[i] = HumanMultimodalMessage(content=f"New camera picture. {text}", images=picture.images)


def compress_history(messages, old_tasks=5, steps=8, pictures=1):
    """Return a shorter copy of the conversation, so it fits the model's context.

    Finished tasks keep only the user's request and the final answer. The task
    in progress keeps its last `steps` tool calls and its newest `pictures`
    pictures. Only the last `old_tasks` finished tasks are kept.
    """
    requests = [i for i, m in enumerate(messages) if is_request(m)]
    if not requests:
        return list(messages)
    system = [m for m in messages[:1] if isinstance(m, SystemMessage)]

    # Finished tasks: request and final answer (an AI message without tool calls)
    old = [m for m in messages[:requests[-1]]
           if is_request(m) or (isinstance(m, AIMessage) and not m.tool_calls)]
    old_requests = [i for i, m in enumerate(old) if is_request(m)]
    if len(old_requests) > old_tasks:
        old = old[old_requests[-old_tasks]:]

    # Task in progress: cut at a tool call so every tool result stays after its call
    now = list(messages[requests[-1]:])
    calls = [i for i, m in enumerate(now) if isinstance(m, AIMessage) and m.tool_calls]
    if len(calls) > steps:
        now = now[:1] + now[calls[-steps]:]
    shots = [i for i, m in enumerate(now) if isinstance(m, HumanMultimodalMessage)]
    for i in reversed(shots[:-pictures]):
        del now[i]
    caption_picture(now)

    return system + old + now
