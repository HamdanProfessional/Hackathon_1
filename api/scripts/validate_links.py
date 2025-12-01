#!/usr/bin/env python3
"""
Link Validation Script

Validates all markdown links in the documentation to ensure:
1. Relative links point to existing files
2. No broken internal references
3. External URLs are formatted correctly

Scans all .md files in web/docs/ and reports any broken links.
"""

import os
import re
import sys
from pathlib import Path
from typing import List, Tuple, Dict
from urllib.parse import urlparse

# Fix encoding for Windows
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Configuration
DOCS_DIR = Path(__file__).parent.parent.parent / "web" / "docs"
MARKDOWN_PATTERN = re.compile(r'\[([^\]]+)\]\(([^)]+)\)')
REPORT_FILE = Path(__file__).parent.parent.parent / "link_validation_report.txt"


class LinkValidator:
    """Validates links in markdown documentation"""

    def __init__(self, docs_dir: Path):
        self.docs_dir = docs_dir
        self.errors: List[Tuple[Path, int, str, str]] = []
        self.warnings: List[Tuple[Path, int, str, str]] = []
        self.total_links = 0
        self.valid_links = 0

    def validate_all(self) -> bool:
        """Validate all markdown files in docs directory"""
        print(f"🔍 Scanning documentation in {self.docs_dir}")
        print("-" * 60)

        if not self.docs_dir.exists():
            print(f"❌ Documentation directory not found: {self.docs_dir}")
            return False

        # Find all markdown files
        md_files = list(self.docs_dir.glob("**/*.md"))
        print(f"📄 Found {len(md_files)} markdown files\n")

        # Validate each file
        for md_file in md_files:
            self.validate_file(md_file)

        # Print results
        self.print_report()
        return len(self.errors) == 0

    def validate_file(self, file_path: Path) -> None:
        """Validate all links in a single markdown file"""
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception as e:
            self.errors.append((file_path, 0, "File Read Error", str(e)))
            return

        # Find all links
        lines = content.split("\n")
        for line_num, line in enumerate(lines, 1):
            for match in MARKDOWN_PATTERN.finditer(line):
                text = match.group(1)
                link = match.group(2)
                self.validate_link(file_path, line_num, text, link)

    def validate_link(
        self, file_path: Path, line_num: int, text: str, link: str
    ) -> None:
        """Validate a single link"""
        self.total_links += 1

        # Skip external links
        if link.startswith("http://") or link.startswith("https://"):
            if self.is_valid_url(link):
                self.valid_links += 1
            else:
                self.warnings.append(
                    (file_path, line_num, f"Invalid URL format: {link}", text)
                )
            return

        # Skip anchors and special links
        if link.startswith("#"):
            self.valid_links += 1
            return

        if link.startswith("mailto:") or link.startswith("tel:"):
            self.valid_links += 1
            return

        # Validate relative links
        if self.is_relative_link(link):
            target_path = self.resolve_relative_path(file_path, link)
            if target_path.exists():
                self.valid_links += 1
            else:
                self.errors.append(
                    (file_path, line_num, f"File not found: {target_path}", text)
                )
        else:
            self.warnings.append((file_path, line_num, f"Invalid link format: {link}", text))

    def is_relative_link(self, link: str) -> bool:
        """Check if link is a relative file path"""
        # Should not be empty and not be a URL or anchor
        return bool(link) and not link.startswith(("/", "http", "#", "mailto", "tel"))

    def is_valid_url(self, url: str) -> bool:
        """Validate URL format"""
        try:
            result = urlparse(url)
            return all([result.scheme in ("http", "https"), result.netloc])
        except Exception:
            return False

    def resolve_relative_path(self, current_file: Path, relative_link: str) -> Path:
        """Resolve a relative link path"""
        # Remove query strings and fragments
        link_path = relative_link.split("#")[0].split("?")[0]

        # Current file directory
        current_dir = current_file.parent

        # Resolve the link
        target = (current_dir / link_path).resolve()

        # Also check if it's relative to docs root
        if not target.exists():
            target = (self.docs_dir / link_path).resolve()

        return target

    def print_report(self) -> None:
        """Print validation report"""
        print("\n" + "=" * 60)
        print("📋 LINK VALIDATION REPORT")
        print("=" * 60)

        # Summary
        print(f"\n📊 Summary:")
        print(f"  Total links scanned: {self.total_links}")
        print(f"  Valid links: {self.valid_links}")
        print(f"  Broken links: {len(self.errors)}")
        print(f"  Warnings: {len(self.warnings)}")

        # Errors
        if self.errors:
            print(f"\n❌ ERRORS ({len(self.errors)}):")
            print("-" * 60)
            for file_path, line_num, error, text in self.errors:
                rel_path = file_path.relative_to(self.docs_dir.parent)
                print(f"  {rel_path}:{line_num}")
                print(f"    Error: {error}")
                print(f"    Text: [{text}]")
                print()

        # Warnings
        if self.warnings:
            print(f"\n⚠️  WARNINGS ({len(self.warnings)}):")
            print("-" * 60)
            for file_path, line_num, warning, text in self.warnings:
                rel_path = file_path.relative_to(self.docs_dir.parent)
                print(f"  {rel_path}:{line_num}")
                print(f"    Warning: {warning}")
                print(f"    Text: [{text}]")
                print()

        # Final status
        print("=" * 60)
        if self.errors:
            print("❌ VALIDATION FAILED: Broken links found!")
            sys.exit(1)
        else:
            print("✅ VALIDATION PASSED: All links are valid!")
            sys.exit(0)


def main():
    """Main entry point"""
    validator = LinkValidator(DOCS_DIR)
    success = validator.validate_all()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
