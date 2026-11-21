"""CLI entry point for the robot agent."""

import asyncio
import json
import sys


async def main():
    from .robot_agent import RobotAgent

    agent = RobotAgent()

    def on_tool_call(name, args, result):
        # Truncate long results for display
        short = result[:200] + "..." if len(result) > 200 else result
        print(f"  [tool] {name}({json.dumps(args)[:100]}) -> {short}")

    try:
        await agent.connect()
        if len(sys.argv) > 1:
            # One-shot mode
            query = " ".join(sys.argv[1:])
            response = await agent.chat(query, on_tool_call=on_tool_call)
            print(response)
        else:
            # Interactive mode
            print("Robot Agent (type 'quit' to exit, 'clear' to reset history)")
            while True:
                try:
                    user_input = input("\n> ")
                except EOFError:
                    break
                command = user_input.strip().lower()
                if command in ("quit", "exit"):
                    break
                if command == "clear":
                    agent.clear_history()
                    print("History cleared.")
                    continue
                if not command:
                    continue
                response = await agent.chat(user_input, on_tool_call=on_tool_call)
                print(response)
    finally:
        await agent.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
