# Embedding Pipeline

This pipeline crawls book URLs, extracts clean text, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval.

## Setup

1. Install dependencies:
   ```bash
   cd backend
   uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
   # Or if using pip:
   pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
   ```

2. Create a `.env` file in the backend directory with your configuration:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here  # Optional for local, required for cloud
   QDRANT_API_KEY=your_qdrant_api_key_here  # Required for cloud
   TARGET_URL=https://physical-ai-humanoid-roboticbook-ha-three.vercel.app/
   CHUNK_SIZE=512
   CHUNK_OVERLAP=50
   ```

## Usage

Run the complete pipeline:
```bash
cd backend
python main.py
```

## Configuration Options

- `COHERE_API_KEY`: Your Cohere API key (required)
- `QDRANT_URL`: Qdrant cluster URL (required for cloud, optional for local)
- `QDRANT_API_KEY`: Qdrant API key (required for cloud)
- `TARGET_URL`: Base URL to crawl (default: the book URL)
- `CHUNK_SIZE`: Size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50)