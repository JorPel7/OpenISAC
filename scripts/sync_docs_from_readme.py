#!/usr/bin/env python3
"""Sync README reference sections into Astro-generated data."""

from __future__ import annotations

import json
import re
from html import unescape
from pathlib import Path
from typing import Any

from markdown_it import MarkdownIt


ROOT = Path(__file__).resolve().parents[1]
GENERATED_JSON = ROOT / "site" / "src" / "generated" / "readme_sections.json"


SYNC_JOBS = (
    {
        "locale": "en",
        "readme": ROOT / "README.md",
        "start_heading": "## Hardware Setup",
    },
    {
        "locale": "zh",
        "readme": ROOT / "README_zh.md",
        "start_heading": "## 硬件准备",
    },
)


def _extract_markdown_section(md_text: str, start_heading: str) -> str:
    lines = md_text.splitlines()
    start_idx = -1
    for i, line in enumerate(lines):
        if line.strip() == start_heading:
            start_idx = i
            break
    if start_idx < 0:
        raise ValueError(f"Heading not found: {start_heading}")
    return "\n".join(lines[start_idx:]).strip() + "\n"


def _slugify_heading(heading_html: str) -> str:
    text = re.sub(r"<[^>]+>", "", heading_html)
    text = unescape(text).strip().lower()
    text = re.sub(r"[^0-9a-z\u4e00-\u9fff]+", "-", text)
    text = re.sub(r"-{2,}", "-", text).strip("-")
    return text or "section"


def _add_heading_ids(rendered_html: str) -> str:
    used: dict[str, int] = {}

    def repl(match: re.Match[str]) -> str:
        level = match.group(1)
        inner = match.group(2)
        base = _slugify_heading(inner)
        count = used.get(base, 0)
        used[base] = count + 1
        slug = base if count == 0 else f"{base}-{count + 1}"
        return f'<h{level} id="{slug}">{inner}</h{level}>'

    return re.sub(r"<h([2-4])>(.*?)</h\1>", repl, rendered_html)


def _render_markdown_to_html(md_text: str) -> str:
    md = MarkdownIt("commonmark", {"html": False})
    md.enable("table")
    rendered = md.render(md_text).strip()
    return _add_heading_ids(rendered)


def _extract_heading_outline(rendered_html: str) -> list[dict[str, Any]]:
    outline: list[dict[str, Any]] = []
    for match in re.finditer(r'<h([2-4]) id="([^"]+)">(.*?)</h\1>', rendered_html):
        level = int(match.group(1))
        heading_id = match.group(2)
        title = unescape(re.sub(r"<[^>]+>", "", match.group(3))).strip()
        outline.append({"level": level, "id": heading_id, "title": title})
    return outline


def _build_generated_payload() -> dict[str, dict[str, Any]]:
    payload: dict[str, dict[str, Any]] = {}
    for job in SYNC_JOBS:
        md_text = Path(job["readme"]).read_text(encoding="utf-8")
        section_md = _extract_markdown_section(md_text, job["start_heading"])
        rendered_html = _render_markdown_to_html(section_md)
        payload[job["locale"]] = {
            "start_heading": job["start_heading"],
            "html": rendered_html,
            "outline": _extract_heading_outline(rendered_html),
        }
    return payload


def _write_generated_json(payload: dict[str, dict[str, Any]]) -> None:
    GENERATED_JSON.parent.mkdir(parents=True, exist_ok=True)
    GENERATED_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    payload = _build_generated_payload()
    _write_generated_json(payload)
    print("Synced Astro generated data from README.")


if __name__ == "__main__":
    main()
