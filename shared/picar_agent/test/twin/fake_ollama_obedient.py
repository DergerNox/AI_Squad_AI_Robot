"""A stand-in for Ollama: the chat model does exactly what the last tool result says,
picture questions are answered by colour (color_vision.py). Requests are logged."""
import json
import os
import re
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from color_vision import answer  # noqa: E402

LOG = sys.argv[1]
TARGETS = sys.argv[2].split(",")
state = {"i": 0, "after": None, "steps": 0}


def call(name, **args):
    return {"content": "", "tool_calls": [{"function": {"name": name, "arguments": args}}]}


def decide(messages):
    tool_texts = [m.get("content") or "" for m in messages if m["role"] == "tool"]
    last = tool_texts[-1] if tool_texts else ""
    target = TARGETS[state["i"]]
    nxt = TARGETS[state["i"] + 1] if state["i"] + 1 < len(TARGETS) else ""
    if "call reached now" in last:
        state["after"] = None
        return call("reached", target=target, next_target=nxt)
    if state["after"]:
        step, state["after"] = state["after"], None
        return step
    if not tool_texts:
        return call("look_around", target=target)
    if "finish with DONE" in last:
        return {"content": "DONE All targets reached."}
    if re.search(r"Back up from the .*?, then call look_around for the", last):
        state["i"] += 1
        state["after"] = call("look_around", target=TARGETS[state["i"]])
        return call("drive", distance_m=-0.3)
    turn = re.search(r"Call turn with angle_deg (-?\d+)", last)
    if turn:
        state["after"] = call("approach", target=target)
        return call("turn", angle_deg=int(turn.group(1)))
    if "turn 120 and call look_around again" in last:
        state["after"] = call("look_around", target=target)
        return call("turn", angle_deg=120)
    if re.search(r"[Cc]all approach", last):
        return call("approach", target=target)
    return call("look_around", target=target)


def now():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        body = json.dumps({"version": "0.13.0"}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        req = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        messages = req.get("messages", [])
        if not req.get("tools"):   # a picture question from a tool
            step = {"content": answer(messages[-1].get("content") or "", messages[-1]["images"][0])}
        else:
            step = decide(messages)
            state["steps"] += 1
            with open(LOG, "a") as f:
                f.write(json.dumps({"step": state["steps"], "last_tool": [m.get("content", "")[:200] for m in messages if m["role"] == "tool"][-1:],
                                    "reply": step}) + "\n")
        self.send_response(200)
        self.send_header("Content-Type", "application/x-ndjson")
        self.end_headers()
        if step["content"]:
            chunk = {"model": req["model"], "created_at": now(),
                     "message": {"role": "assistant", "content": step["content"]}, "done": False}
            self.wfile.write((json.dumps(chunk) + "\n").encode())
        if step.get("tool_calls"):
            chunk = {"model": req["model"], "created_at": now(),
                     "message": {"role": "assistant", "content": "", "tool_calls": step["tool_calls"]}, "done": False}
            self.wfile.write((json.dumps(chunk) + "\n").encode())
        final = {"model": req["model"], "created_at": now(), "message": {"role": "assistant", "content": ""},
                 "done": True, "done_reason": "stop", "total_duration": 1000, "load_duration": 10,
                 "prompt_eval_count": 100, "prompt_eval_duration": 100, "eval_count": 10, "eval_duration": 100}
        self.wfile.write((json.dumps(final) + "\n").encode())
        self.wfile.flush()


ThreadingHTTPServer(("127.0.0.1", 11434), H).serve_forever()
