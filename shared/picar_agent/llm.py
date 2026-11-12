"""llm.py: the AI model that drives the car, chosen in .env (no code changes needed).

    PICAR_LLM          ollama (local, the default), openai (OpenAI or any server with an
                       OpenAI-compatible API) or google (Gemini)
    PICAR_MODEL        the model's name; it must understand pictures and call tools
                       (empty: qwen3-vl:4b-instruct for ollama; required for openai and google)
    PICAR_BASE_URL     the server's address, if it is not the default one (ollama, openai)
    PICAR_TEMPERATURE  how much the answers vary, 0 to 2 (empty: 0.2)
    OPENAI_API_KEY / GOOGLE_API_KEY   the key for a cloud model

Every model gets CompactHistory mixed in, so it always sees a shortened conversation
(history.py). The tools ask the same model their questions about pictures.
"""

import os

from history import compress_history

PROVIDERS = ("ollama", "openai", "google")
DEFAULT_OLLAMA_MODEL = "qwen3-vl:4b-instruct"
DEFAULT_TEMPERATURE = 0.2
OLLAMA_CONTEXT = 8192   # tokens; compress_history keeps each request well under it


class CompactHistory:
    """Mixed into a chat model class: the model gets a compressed copy of the conversation."""

    # run_manager must be named here: LangChain only streams the words to the
    # callbacks when the method's signature has it.
    def _generate(self, messages, stop=None, run_manager=None, **kwargs):
        return super()._generate(compress_history(messages), stop, run_manager, **kwargs)

    def _stream(self, messages, stop=None, run_manager=None, **kwargs):
        return super()._stream(compress_history(messages), stop, run_manager, **kwargs)


def settings():
    """The model settings from the environment (.env), checked.
    Returns provider, model, base_url (None: the default) and temperature."""
    provider = os.environ.get("PICAR_LLM", "").strip().lower() or "ollama"
    model = os.environ.get("PICAR_MODEL", "").strip()
    base_url = os.environ.get("PICAR_BASE_URL", "").strip() or None
    temperature_text = os.environ.get("PICAR_TEMPERATURE", "").strip()
    if provider not in PROVIDERS:
        raise ValueError(f"PICAR_LLM must be ollama, openai or google, not '{provider}'")
    if provider != "ollama" and not model:
        raise ValueError(f"Set PICAR_MODEL in .env: the {provider} model to use")
    try:
        temperature = float(temperature_text) if temperature_text else DEFAULT_TEMPERATURE
    except ValueError:
        raise ValueError(f"PICAR_TEMPERATURE must be a number, not '{temperature_text}'") from None
    return provider, model or DEFAULT_OLLAMA_MODEL, base_url, temperature


def ollama_model(model, base_url, temperature):
    from langchain_ollama import ChatOllama

    class CompactChatOllama(CompactHistory, ChatOllama):
        pass

    return CompactChatOllama(model=model, base_url=base_url, num_ctx=OLLAMA_CONTEXT,
                             temperature=temperature)


def openai_model(model, base_url, temperature):   # the key comes from OPENAI_API_KEY
    from langchain_openai import ChatOpenAI

    class CompactChatOpenAI(CompactHistory, ChatOpenAI):
        pass

    return CompactChatOpenAI(model=model, base_url=base_url, temperature=temperature)


def google_model(model, base_url, temperature):   # the key comes from GOOGLE_API_KEY
    from langchain_google_genai import ChatGoogleGenerativeAI

    class CompactGemini(CompactHistory, ChatGoogleGenerativeAI):
        pass

    return CompactGemini(model=model, temperature=temperature)   # Google's own address


MAKERS = {"ollama": ollama_model, "openai": openai_model, "google": google_model}


def make_llm():
    """The chat model chosen in .env. Returns the model and a short description."""
    provider, model, base_url, temperature = settings()
    if provider == "google":
        base_url = None   # Gemini is always at Google's own address
    llm = MAKERS[provider](model, base_url, temperature)
    description = f"{provider}, {model}" + (f" at {base_url}" if base_url else "")
    if temperature != DEFAULT_TEMPERATURE:
        description += f", temperature {temperature:g}"
    return llm, description
