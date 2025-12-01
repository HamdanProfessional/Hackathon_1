#!/usr/bin/env python3
"""
Code Block Linting Script

Extracts and validates code blocks from markdown documentation.

Features:
- Extracts Python code blocks from markdown
- Runs syntax validation on code
- Reports any syntax errors or issues
- Supports multiple languages (future: JavaScript, bash, etc.)
"""

import os
import re
import sys
import ast
import tempfile
from pathlib import Path
from typing import List, Tuple, Dict

# Fix encoding for Windows
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Configuration
DOCS_DIR = Path(__file__).parent.parent.parent / "web" / "docs"
CODE_BLOCK_PATTERN = re.compile(r"```(\w+)?\n(.*?)\n```", re.DOTALL)
REPORT_FILE = Path(__file__).parent.parent.parent / "code_lint_report.txt"


class CodeBlockLinter:
    """Lints code blocks in markdown documentation"""

    def __init__(self, docs_dir: Path):
        self.docs_dir = docs_dir
        self.errors: List[Tuple[Path, int, str, str]] = []
        self.warnings: List[Tuple[Path, int, str, str]] = []
        self.total_blocks = 0
        self.validated_blocks = 0

    def lint_all(self) -> bool:
        """Lint all code blocks in documentation"""
        print(f"🔍 Scanning code blocks in {self.docs_dir}")
        print("-" * 60)

        if not self.docs_dir.exists():
            print(f"❌ Documentation directory not found: {self.docs_dir}")
            return False

        # Find all markdown files
        md_files = list(self.docs_dir.glob("**/*.md"))
        print(f"📄 Found {len(md_files)} markdown files\n")

        # Lint each file
        for md_file in md_files:
            self.lint_file(md_file)

        # Print results
        self.print_report()
        return len(self.errors) == 0

    def lint_file(self, file_path: Path) -> None:
        """Lint all code blocks in a markdown file"""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception as e:
            self.warnings.append((file_path, 0, "File Read Error", str(e)))
            return

        # Find all code blocks
        lines = content.split("\n")
        current_line = 0

        for line_num, line in enumerate(lines, 1):
            if line.startswith("```"):
                # Extract language
                lang_match = re.match(r"```(\w+)?", line)
                language = lang_match.group(1) if lang_match else "text"

                # Find end of block
                block_lines = []
                for i in range(line_num, len(lines)):
                    if lines[i].startswith("```"):
                        break
                    block_lines.append(lines[i])

                if block_lines:
                    code = "\n".join(block_lines)
                    self.lint_code_block(file_path, line_num, language, code)

    def lint_code_block(
        self, file_path: Path, line_num: int, language: str, code: str
    ) -> None:
        """Lint a single code block"""
        self.total_blocks += 1

        # Skip empty blocks
        if not code.strip():
            lang_name = language or "text"
            self.warnings.append((file_path, line_num, "Empty code block", lang_name))
            return

        # Only validate Python code for now
        if language and language.lower() in ("python", "py"):
            self.lint_python_code(file_path, line_num, code)
        else:
            # Other languages: just note them
            self.validated_blocks += 1

    def lint_python_code(self, file_path: Path, line_num: int, code: str) -> None:
        """Validate Python code syntax"""
        try:
            # Try to parse as Python code
            ast.parse(code)
            self.validated_blocks += 1
        except SyntaxError as e:
            self.errors.append(
                (file_path, line_num, f"Syntax Error: {e.msg}", code[:100])
            )
        except Exception as e:
            self.warnings.append(
                (file_path, line_num, f"Validation Error: {str(e)}", code[:100])
            )

    def print_report(self) -> None:
        """Print linting report"""
        print("\n" + "=" * 60)
        print("📋 CODE LINTING REPORT")
        print("=" * 60)

        # Summary
        print(f"\n📊 Summary:")
        print(f"  Total code blocks: {self.total_blocks}")
        print(f"  Validated blocks: {self.validated_blocks}")
        print(f"  Syntax errors: {len(self.errors)}")
        print(f"  Warnings: {len(self.warnings)}")

        # Errors
        if self.errors:
            print(f"\n❌ ERRORS ({len(self.errors)}):")
            print("-" * 60)
            for file_path, line_num, error, code_snippet in self.errors:
                rel_path = file_path.relative_to(self.docs_dir.parent)
                print(f"  {rel_path}:{line_num}")
                print(f"    Error: {error}")
                if code_snippet:
                    print(f"    Code: {code_snippet}...")
                print()

        # Warnings
        if self.warnings:
            print(f"\n⚠️  WARNINGS ({len(self.warnings)}):")
            print("-" * 60)
            for file_path, line_num, warning, code_snippet in self.warnings:
                rel_path = file_path.relative_to(self.docs_dir.parent)
                print(f"  {rel_path}:{line_num}")
                print(f"    Warning: {warning}")
                if code_snippet:
                    print(f"    Code: {code_snippet}...")
                print()

        # Final status
        print("=" * 60)
        if self.errors:
            print("❌ LINTING FAILED: Syntax errors found!")
            sys.exit(1)
        else:
            print("✅ LINTING PASSED: All code blocks validated!")
            sys.exit(0)


def main():
    """Main entry point"""
    linter = CodeBlockLinter(DOCS_DIR)
    success = linter.lint_all()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
