"""compress_history and caption_picture: what the model gets to see of the conversation."""

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from rai.messages import HumanMultimodalMessage

from picar_tools import caption_picture, compress_history

IMAGE = "iVBORw0KGgo="   # stands in for a base64 picture; never decoded here


def text_of(message):
    """The text of a message. Picture messages keep it as a list of parts."""
    if isinstance(message.content, str):
        return message.content
    return " ".join(part["text"] for part in message.content if part.get("type") == "text")


def step(call_id, name="drive"):
    """One tool call by the model, its result, and the picture that comes with it."""
    return [
        AIMessage(content="", tool_calls=[{"name": name, "args": {}, "id": call_id}]),
        ToolMessage(content=f"result {call_id}", tool_call_id=call_id),
        HumanMultimodalMessage(content="Image returned by a tool call", images=[IMAGE]),
    ]


def task(name, steps, answer="DONE"):
    """A finished task: the request, some steps and the final answer."""
    messages = [HumanMessage(content=name)]
    for n in range(steps):
        messages += step(f"{name}-{n}")
    return messages + [AIMessage(content=f"{answer} {name}")]


def running(name, steps):
    """The task in progress: the request and some steps, no answer yet."""
    return task(name, steps)[:-1]


SYSTEM = SystemMessage(content="You drive a small robot car.")


def test_without_a_request_the_conversation_is_unchanged():
    messages = [SYSTEM]
    result = compress_history(messages)
    assert result == messages and result is not messages


def test_finished_tasks_keep_only_the_request_and_the_answer():
    result = compress_history([SYSTEM] + task("task 1", 3) + running("task 2", 0))
    assert [text_of(m) for m in result] == [SYSTEM.content, "task 1", "DONE task 1", "task 2"]


def test_only_the_last_five_finished_tasks_are_kept():
    messages = [SYSTEM]
    for n in range(1, 8):
        messages += task(f"task {n}", 1)
    result = compress_history(messages + running("task 8", 0))
    requests = [text_of(m) for m in result if type(m) is HumanMessage]
    assert requests == ["task 3", "task 4", "task 5", "task 6", "task 7", "task 8"]


def test_the_task_in_progress_keeps_its_last_eight_tool_calls():
    result = compress_history([SYSTEM] + running("now", 12))
    calls = [m.tool_calls[0]["id"] for m in result if isinstance(m, AIMessage) and m.tool_calls]
    assert calls == [f"now-{n}" for n in range(4, 12)]
    assert text_of(result[1]) == "now"   # the request itself stays


def test_every_tool_result_stays_after_its_call():
    result = compress_history([SYSTEM] + task("old", 4) + running("now", 12))
    called = set()
    for message in result:
        if isinstance(message, AIMessage):
            called.update(call["id"] for call in message.tool_calls)
        if isinstance(message, ToolMessage):
            assert message.tool_call_id in called


def test_only_the_newest_picture_is_kept_and_it_gets_the_tool_result():
    result = compress_history([SYSTEM] + running("now", 3))
    pictures = [m for m in result if isinstance(m, HumanMultimodalMessage)]
    assert len(pictures) == 1 and result[-1] is pictures[0]
    assert text_of(pictures[0]) == "New camera picture. result now-2"
    assert pictures[0].images == [IMAGE]


def test_a_nudge_is_not_a_new_task():
    messages = [SYSTEM] + running("now", 2) + [HumanMessage(content="Continue.", name="nudge")]
    messages += step("now-2")
    result = compress_history(messages)
    requests = [text_of(m) for m in result if type(m) is HumanMessage]
    assert requests == ["now", "Continue."]   # the nudge stays inside the task
    assert sum(isinstance(m, ToolMessage) for m in result) == 3


def test_caption_joins_a_result_stored_as_parts():
    messages = [
        AIMessage(content="", tool_calls=[{"name": "drive", "args": {}, "id": "c1"}]),
        ToolMessage(content=[{"type": "text", "text": "Drove +0.50 m."}], tool_call_id="c1"),
        HumanMultimodalMessage(content="Image returned by a tool call", images=[IMAGE]),
    ]
    caption_picture(messages)
    assert text_of(messages[-1]) == "New camera picture. Drove +0.50 m."


def test_caption_leaves_a_conversation_alone_when_the_last_message_is_not_a_picture():
    messages = step("c1")[:2]
    before = list(messages)
    caption_picture(messages)
    assert messages == before
