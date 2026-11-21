"""Claude API and MCP tool loop shared by the CLI and GUI."""

import json
import logging
import os
from contextlib import AsyncExitStack
from pathlib import Path
from typing import Awaitable, Callable, Optional

from mcp.client.session import ClientSession
from mcp.client.stdio import stdio_client, StdioServerParameters

logger = logging.getLogger(__name__)

# Route caller-provided tools through extra_dispatch instead of MCP.
_LOCAL_SESSION_SENTINEL = "__local__"

# Shared by the CLI, GUI and robot-operation skills; read for each new agent.
_PROMPT_PATH = Path(__file__).absolute().with_name("robot_operation.md")


def _create_client():
    """Create a Hermes client using AIFAPIM_API_KEY.

    HERMES_ENDPOINT and BEAMBOT_MODEL override the gateway and model.
    """
    from anthropic import Anthropic

    endpoint = os.environ.get("HERMES_ENDPOINT", "https://hermes.nsls2.bnl.gov")
    client = Anthropic(
        base_url=f"{endpoint}/anthropic",
        api_key=os.environ["AIFAPIM_API_KEY"],
    )
    return client, os.environ.get("BEAMBOT_MODEL", "claude-sonnet-4-6")


class RobotAgent:
    """Manage model requests, MCP tools and conversation history."""

    def __init__(
        self,
        model=None,
        mcp_config_path=None,
        system_prompt_prefix: str = "",
        extra_tools: Optional[list] = None,
        extra_dispatch: Optional[Callable[[str, dict], Awaitable[str]]] = None,
        tool_filter: Optional[Callable[[str, str], bool]] = None,
    ):
        self.client, default_model = _create_client()
        self.model = model or default_model
        base_prompt = _PROMPT_PATH.read_text()
        self.system_prompt = (
            f"{system_prompt_prefix}\n\n---\n\n{base_prompt}"
            if system_prompt_prefix
            else base_prompt
        )
        self.tools = []  # Anthropic API tool format
        self.tool_to_session = {}  # tool_name -> ClientSession or sentinel
        self.messages = []
        self._exit_stack = AsyncExitStack()
        self._mcp_config_path = mcp_config_path
        self._extra_tools = list(extra_tools or [])
        self._extra_dispatch = extra_dispatch
        self._tool_filter = tool_filter

    async def connect(self):
        """Connect configured MCP servers and register caller-provided tools."""
        config_path = self._mcp_config_path or self._find_mcp_config()
        if not config_path:
            raise FileNotFoundError("No .mcp.json found")

        with open(config_path) as f:
            config = json.load(f)

        await self._exit_stack.__aenter__()

        for name, server_cfg in config.get("mcpServers", {}).items():
            try:
                await self._connect_server(name, server_cfg)
            except Exception as e:
                logger.warning(f"Failed to connect to MCP server '{name}': {e}")

        # Local tools bypass MCP and use the caller's dispatcher.
        for tool in self._extra_tools:
            self.tools.append(tool)
            self.tool_to_session[tool["name"]] = _LOCAL_SESSION_SENTINEL

        logger.info(
            f"Connected: {len(self.tools)} tools "
            f"({len(self._extra_tools)} local) from "
            f"{len({s for s in self.tool_to_session.values() if s != _LOCAL_SESSION_SENTINEL})} server(s)"
        )

    async def _connect_server(self, name, server_cfg):
        """Connect to a single MCP server and register its tools."""
        params = StdioServerParameters(
            command=server_cfg["command"],
            args=server_cfg.get("args", []),
            env=server_cfg.get("env"),
        )

        # Keep transports and sessions open until disconnect().
        streams = await self._exit_stack.enter_async_context(stdio_client(params))
        read_stream, write_stream = streams

        session = await self._exit_stack.enter_async_context(
            ClientSession(read_stream, write_stream)
        )
        await session.initialize()

        result = await session.list_tools()
        registered = 0
        for tool in result.tools:
            if self._tool_filter and not self._tool_filter(name, tool.name):
                logger.debug(f"  {name}: filtered out '{tool.name}'")
                continue
            self.tools.append(
                {
                    "name": tool.name,
                    "description": tool.description or "",
                    "input_schema": tool.inputSchema,
                }
            )
            self.tool_to_session[tool.name] = session
            registered += 1

        logger.info(f"  {name}: {registered}/{len(result.tools)} tools")

    async def chat(self, user_message: str, on_tool_call=None, on_text=None) -> str:
        """Run the model/tool loop and return the final text.

        on_tool_call(name, input, result) runs after each tool call.
        on_text(text) receives the final response, not streamed chunks.
        """
        self.messages.append({"role": "user", "content": user_message})

        while True:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=4096,
                system=self.system_prompt,
                tools=self.tools,
                messages=self.messages,
            )

            content = []
            for block in response.content:
                if block.type == "text":
                    content.append({"type": "text", "text": block.text})
                elif block.type == "tool_use":
                    content.append(
                        {
                            "type": "tool_use",
                            "id": block.id,
                            "name": block.name,
                            "input": block.input,
                        }
                    )
            self.messages.append({"role": "assistant", "content": content})

            if response.stop_reason == "end_turn":
                text = "".join(b.text for b in response.content if b.type == "text")
                if on_text:
                    on_text(text)
                return text

            tool_results = []
            for block in response.content:
                if block.type == "tool_use":
                    result_text = await self._call_tool(block.name, block.input)
                    if on_tool_call:
                        on_tool_call(block.name, block.input, result_text)
                    tool_results.append(
                        {
                            "type": "tool_result",
                            "tool_use_id": block.id,
                            "content": result_text,
                        }
                    )

            if tool_results:
                self.messages.append({"role": "user", "content": tool_results})

    async def _call_tool(self, name: str, arguments: dict) -> str:
        """Dispatch a tool call to MCP or the caller's local handler."""
        session = self.tool_to_session.get(name)
        if not session:
            return f"Error: tool '{name}' not found in any connected MCP server"

        if session == _LOCAL_SESSION_SENTINEL:
            if self._extra_dispatch is None:
                return f"Error: tool '{name}' is local but no dispatcher is configured"
            try:
                return await self._extra_dispatch(name, arguments)
            except Exception as e:
                return f"Error in local dispatch for '{name}': {e}"

        try:
            result = await session.call_tool(name, arguments)
            parts = []
            for item in result.content:
                if hasattr(item, "text"):
                    parts.append(item.text)
            return "\n".join(parts) if parts else str(result)
        except Exception as e:
            return f"Error calling tool '{name}': {e}"

    def clear_history(self):
        """Reset conversation history."""
        self.messages = []

    async def disconnect(self):
        """Shut down all MCP server connections."""
        await self._exit_stack.aclose()

    def _find_mcp_config(self):
        """Find .mcp.json within ten directory levels, starting at cwd."""
        path = Path.cwd()
        for _ in range(10):
            candidate = path / ".mcp.json"
            if candidate.exists():
                return str(candidate)
            parent = path.parent
            if parent == path:
                break
            path = parent
        return None
