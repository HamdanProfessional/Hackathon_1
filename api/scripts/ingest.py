"""
Content Ingestion Script for Qdrant Vector Database
Recursively reads markdown files from web/docs/en/ and creates embeddings
"""
import os
import sys
from pathlib import Path
from typing import List, Dict
import re

# Add parent directory to path to import from src
sys.path.append(str(Path(__file__).parent.parent))

from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from dotenv import load_dotenv

# Load environment variables
load_dotenv(Path(__file__).parent.parent.parent / ".env")


class TextbookIngestion:
    """Handles ingestion of textbook content into Qdrant"""

    def __init__(self):
        # Initialize OpenAI client for embeddings
        self.openai_client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY")
        )

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )

        self.collection_name = "robotics_textbook"
        self.embedding_model = "text-embedding-3-small"

    def create_collection(self):
        """Create Qdrant collection if it doesn't exist"""
        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections().collections
            if any(c.name == self.collection_name for c in collections):
                print(f"✓ Collection '{self.collection_name}' already exists")
                return

            # Create collection with 1536 dimensions (text-embedding-3-small)
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
            )
            print(f"✓ Created collection '{self.collection_name}'")

        except Exception as e:
            print(f"✗ Error creating collection: {e}")
            raise

    def chunk_text(self, content: str, max_chars: int = 500) -> List[str]:
        """
        Chunk text by headers or fixed character windows

        Args:
            content: Markdown content
            max_chars: Maximum characters per chunk

        Returns:
            List of text chunks
        """
        chunks = []

        # Split by headers (## or ###)
        header_pattern = r'^(#{2,3})\s+(.+)$'
        sections = re.split(header_pattern, content, flags=re.MULTILINE)

        current_chunk = ""
        current_header = ""

        for i, section in enumerate(sections):
            # Check if it's a header marker
            if re.match(r'^#{2,3}$', section):
                continue
            # Check if it's a header title
            elif i > 0 and re.match(r'^#{2,3}$', sections[i-1]):
                current_header = section.strip()
                continue
            # It's content
            else:
                # Add header to chunk if we have one
                section_text = section.strip()
                if not section_text:
                    continue

                if current_header:
                    section_text = f"{current_header}\n\n{section_text}"

                # If chunk is too long, split by character windows
                if len(section_text) > max_chars:
                    # Split into windows
                    for j in range(0, len(section_text), max_chars):
                        chunk = section_text[j:j + max_chars]
                        if chunk.strip():
                            chunks.append(chunk.strip())
                else:
                    chunks.append(section_text)

                current_header = ""

        return [c for c in chunks if len(c) > 50]  # Filter very small chunks

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using OpenAI text-embedding-3-small

        Args:
            text: Text to embed

        Returns:
            Embedding vector
        """
        response = self.openai_client.embeddings.create(
            model=self.embedding_model,
            input=text
        )
        return response.data[0].embedding

    def read_markdown_files(self, docs_path: Path) -> List[Dict]:
        """
        Recursively read all markdown files

        Args:
            docs_path: Path to docs/en/ directory

        Returns:
            List of documents with metadata
        """
        documents = []

        for md_file in docs_path.rglob("*.md"):
            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract metadata from frontmatter
                metadata = {
                    "filename": str(md_file.relative_to(docs_path.parent.parent)),
                    "module": md_file.parent.name if md_file.parent.name != "en" else "intro"
                }

                # Extract title from frontmatter if exists
                title_match = re.search(r'^title:\s*(.+)$', content, re.MULTILINE)
                if title_match:
                    metadata["title"] = title_match.group(1).strip()

                documents.append({
                    "content": content,
                    "metadata": metadata
                })

                print(f"✓ Read: {metadata['filename']}")

            except Exception as e:
                print(f"✗ Error reading {md_file}: {e}")

        return documents

    def ingest(self, docs_path: str):
        """
        Main ingestion pipeline

        Args:
            docs_path: Path to web/docs/en/ directory
        """
        docs_dir = Path(docs_path)
        if not docs_dir.exists():
            raise FileNotFoundError(f"Directory not found: {docs_path}")

        print(f"\n🚀 Starting ingestion from: {docs_dir}")
        print("=" * 60)

        # Step 1: Create collection
        self.create_collection()

        # Step 2: Read markdown files
        print(f"\n📖 Reading markdown files...")
        documents = self.read_markdown_files(docs_dir)
        print(f"✓ Found {len(documents)} documents")

        # Step 3: Chunk and embed
        print(f"\n✂️  Chunking and embedding...")
        points = []
        point_id = 0

        for doc in documents:
            chunks = self.chunk_text(doc["content"])
            print(f"  → {doc['metadata']['filename']}: {len(chunks)} chunks")

            for chunk in chunks:
                # Generate embedding
                embedding = self.generate_embedding(chunk)

                # Create point
                points.append(PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        **doc["metadata"],
                        "text": chunk
                    }
                ))
                point_id += 1

        # Step 4: Upsert to Qdrant
        print(f"\n💾 Upserting {len(points)} vectors to Qdrant...")
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            print(f"  → Uploaded batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1}")

        print(f"\n✅ Ingestion complete! {len(points)} vectors stored.")
        print(f"   Collection: {self.collection_name}")
        print("=" * 60)


def main():
    """Main entry point"""
    # Determine web/docs/en/ path relative to this script
    script_dir = Path(__file__).parent
    web_docs_en = script_dir.parent.parent / "web" / "docs" / "en"

    if not web_docs_en.exists():
        print(f"✗ Error: {web_docs_en} does not exist")
        print("   Please ensure you've generated the textbook content first.")
        sys.exit(1)

    try:
        ingestion = TextbookIngestion()
        ingestion.ingest(str(web_docs_en))
    except Exception as e:
        print(f"\n✗ Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
