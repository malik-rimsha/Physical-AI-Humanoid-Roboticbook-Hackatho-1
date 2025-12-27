# Quickstart Guide: Embedding Pipeline

## Prerequisites

- Python 3.9 or higher
- UV package manager
- Cohere API key
- Qdrant cluster (local or cloud)

## Setup

### 1. Clone and Navigate to Backend Directory
```bash
mkdir backend
cd backend
```

### 2. Initialize Project with UV
```bash
uv init
uv add requests beautifulsoup4 cohere qdrant-client python-dotenv
```

### 3. Environment Configuration
Create a `.env` file in the backend directory:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here  # Optional for local, required for cloud
QDRANT_API_KEY=your_qdrant_api_key_here  # Required for cloud
TARGET_URL=https://physical-ai-humanoid-roboticbook-ha-three.vercel.app/
```

## Usage

### 1. Run the Complete Pipeline
```bash
python main.py
```

### 2. Pipeline Steps
The pipeline will execute the following steps:
1. Fetch all URLs from the target site
2. Extract clean text from each URL
3. Chunk the text into appropriate sizes
4. Generate embeddings using Cohere
5. Create the 'rag-embadding' collection in Qdrant
6. Save each chunk with its embedding to Qdrant

## Configuration Options

### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key (required)
- `QDRANT_URL`: Qdrant cluster URL (required for cloud, optional for local)
- `QDRANT_API_KEY`: Qdrant API key (required for cloud)
- `TARGET_URL`: Base URL to crawl (default: the book URL)
- `CHUNK_SIZE`: Size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50)

## Expected Output

After running the pipeline:
- All text content from the target URLs will be processed
- Embeddings will be stored in the 'rag-embadding' Qdrant collection
- Each chunk will be stored with its metadata for retrieval
- Processing logs will show progress and any errors