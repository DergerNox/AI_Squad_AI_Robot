"""llm.py: the model chosen in .env, without calling any model (no network needed)."""

import pytest
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolMessage
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_ollama import ChatOllama
from langchain_openai import ChatOpenAI

import llm
from llm import CompactHistory, make_llm, settings

SETTINGS = ("PICAR_LLM", "PICAR_MODEL", "PICAR_BASE_URL", "PICAR_TEMPERATURE")


@pytest.fixture(autouse=True)
def env(monkeypatch):
    """No PICAR_ settings from the shell, and stand-in keys (never a real one)."""
    for name in SETTINGS:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("OPENAI_API_KEY", "not-a-real-key")
    monkeypatch.setenv("GOOGLE_API_KEY", "not-a-real-key")
    return monkeypatch


# ---------- settings ----------

def test_nothing_set_means_qwen_in_ollama():
    assert settings() == ("ollama", "qwen3-vl:4b-instruct", None, 0.2)


@pytest.mark.parametrize("written", ["", "  ", "ollama", " Ollama "])
def test_empty_or_any_case_ollama_means_ollama(env, written):
    env.setenv("PICAR_LLM", written)
    assert settings()[0] == "ollama"


def test_cloud_providers_need_a_model_name(env):
    for provider in ("openai", "google"):
        env.setenv("PICAR_LLM", provider)
        with pytest.raises(ValueError, match=f"Set PICAR_MODEL in .env: the {provider} model"):
            settings()


def test_an_unknown_provider_says_which_ones_there_are(env):
    env.setenv("PICAR_LLM", "claude")
    with pytest.raises(ValueError, match="PICAR_LLM must be ollama, openai or google, not 'claude'"):
        settings()


def test_temperature_can_be_set(env):
    env.setenv("PICAR_TEMPERATURE", "1.0")
    assert settings()[3] == 1.0
    env.setenv("PICAR_TEMPERATURE", "warm")
    with pytest.raises(ValueError, match="PICAR_TEMPERATURE must be a number, not 'warm'"):
        settings()


# ---------- the models ----------

def test_ollama(env):
    env.setenv("PICAR_BASE_URL", "http://192.168.1.20:11434")
    model, description = make_llm()
    assert isinstance(model, CompactHistory) and isinstance(model, ChatOllama)
    assert (model.model, model.base_url, model.num_ctx, model.temperature) == (
        "qwen3-vl:4b-instruct", "http://192.168.1.20:11434", 8192, 0.2)
    assert description == "ollama, qwen3-vl:4b-instruct at http://192.168.1.20:11434"


def test_openai_or_a_compatible_server(env):
    env.setenv("PICAR_LLM", "openai")
    env.setenv("PICAR_MODEL", "some-model")
    env.setenv("PICAR_BASE_URL", "https://example.invalid/v1")
    model, description = make_llm()
    assert isinstance(model, CompactHistory) and isinstance(model, ChatOpenAI)
    assert (model.model_name, model.openai_api_base, model.temperature) == (
        "some-model", "https://example.invalid/v1", 0.2)
    assert description == "openai, some-model at https://example.invalid/v1"


def test_gemini_ignores_the_server_address(env):
    env.setenv("PICAR_LLM", "google")
    env.setenv("PICAR_MODEL", "gemini-3.6-flash")
    env.setenv("PICAR_BASE_URL", "http://192.168.1.20:11434")   # left over from Ollama
    env.setenv("PICAR_TEMPERATURE", "1")
    model, description = make_llm()
    assert isinstance(model, CompactHistory) and isinstance(model, ChatGoogleGenerativeAI)
    assert (model.model, model.temperature) == ("gemini-3.6-flash", 1.0)
    assert description == "google, gemini-3.6-flash, temperature 1"


def test_every_provider_has_a_maker():
    assert set(llm.MAKERS) == set(llm.PROVIDERS)


@pytest.mark.parametrize("provider", ["ollama", "openai", "google"])
def test_every_provider_gets_the_temperature(env, provider):
    env.setenv("PICAR_LLM", provider)
    env.setenv("PICAR_MODEL", "some-model")
    env.setenv("PICAR_TEMPERATURE", "0.7")
    model, description = make_llm()
    assert model.temperature == 0.7 and description.endswith(", temperature 0.7")


# ---------- CompactHistory ----------

class Recorder:
    """Stands in for a chat model: writes down what it is sent."""

    def _generate(self, messages, stop=None, run_manager=None, **kwargs):
        self.sent = ("generate", messages, stop, run_manager, kwargs)
        return "answer"

    def _stream(self, messages, stop=None, run_manager=None, **kwargs):
        self.sent = ("stream", messages, stop, run_manager, kwargs)
        return iter(["word"])


class CompactRecorder(CompactHistory, Recorder):
    pass


def long_conversation():
    """A finished task with tool calls, then the task in progress."""
    messages = [SystemMessage(content="You drive a car."), HumanMessage(content="task 1")]
    for n in range(3):
        messages += [AIMessage(content="", tool_calls=[{"name": "drive", "args": {}, "id": f"c{n}"}]),
                     ToolMessage(content=f"result {n}", tool_call_id=f"c{n}")]
    return messages + [AIMessage(content="DONE task 1"), HumanMessage(content="task 2")]


@pytest.mark.parametrize("method", ["_generate", "_stream"])
def test_the_model_gets_the_shortened_conversation_and_everything_else(method):
    model = CompactRecorder()
    getattr(model, method)(long_conversation(), stop=["STOP"], run_manager="callbacks", extra=1)
    kind, messages, stop, run_manager, kwargs = model.sent
    assert kind == method.strip("_")
    assert [m.content for m in messages] == ["You drive a car.", "task 1", "DONE task 1", "task 2"]
    assert (stop, run_manager, kwargs) == (["STOP"], "callbacks", {"extra": 1})
