---
tools-used:
  - claude-code
  - codex
models-used:
  - claude-opus-4-8
  - claude-opus-5
  - claude-opus-5-5
  - claude-sonnet-5-5
  - gpt-5.6-sol
  - gpt-5.6-terra
  - gpt-6.0-sol
providers:
  - Anthropic
  - OpenAI
scope:
  human-authored: >
    System design and calibration.
  ai-assisted: >
    beambot framework, robot and end-effector configs, setup scripts,
    and Docker image.
  ai-generated: >
    beambot_gui.
last-updated: 2026-10-06
---

# AI Disclosure

## Scope of AI use

**System design and calibration** are human-authored.
**The beambot framework, robot and end-effector configs, setup
scripts, and Docker image** are ai-assisted with human review and
validation before merging.
**beambot_gui** is ai-generated and reviewed by the developer.

## Tools and models

AI assistance is provided via **Claude Code** and **Codex** using
Anthropic and OpenAI models. Per-commit `Assisted-by: AGENT:MODEL`
trailers are the authoritative per-change record.

## Purpose of use

- Refactor ROS 2 packages and configs.
- Generate the beambot_gui operator interface.

## Input data

Only repository source files, public documentation, and
non-sensitive prompts are provided to AI models. No sensitive,
CUI, export-controlled, or PII data is submitted to external
AI services.

## Limitations

LLM output may contain errors or bias. All AI-assisted content
is independently reviewed and validated before merging.

## Reviewer disclaimer

AI-assisted content in this repository has been reviewed by
the project team. The contributing staff member is responsible
for the correctness of every merged change.
