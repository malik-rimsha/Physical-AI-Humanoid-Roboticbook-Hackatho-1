# Function Interface Contracts

## Core Functions Definition

### `get_all_urls(base_url: str) -> List[str]`
- **Purpose**: Extract all relevant URLs from the target site
- **Input**: Base URL to crawl
- **Output**: List of URLs to process
- **Errors**: May raise exception if base URL is inaccessible

### `extract_text_from_url(url: str) -> Dict[str, str]`
- **Purpose**: Extract clean text content from a single URL
- **Input**: URL to extract text from
- **Output**: Dictionary with 'title' and 'content' keys
- **Errors**: May raise exception if URL is inaccessible or content extraction fails

### `chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[Dict[str, str]]`
- **Purpose**: Split text into smaller chunks for embedding
- **Input**: Text to chunk, chunk size, overlap
- **Output**: List of chunk dictionaries with text and metadata
- **Errors**: None expected under normal conditions

### `embed(text_chunks: List[str]) -> List[List[float]]`
- **Purpose**: Generate embeddings for text chunks using Cohere
- **Input**: List of text chunks to embed
- **Output**: List of embedding vectors
- **Errors**: May raise exception if Cohere API is unavailable

### `create_collection(collection_name: str) -> bool`
- **Purpose**: Initialize Qdrant collection for storing embeddings
- **Input**: Name of collection to create
- **Output**: True if successful, False otherwise
- **Errors**: May raise exception if Qdrant is unavailable

### `save_chunk_to_qdrant(chunk_data: Dict, embedding: List[float]) -> bool`
- **Purpose**: Store a single chunk with its embedding in Qdrant
- **Input**: Chunk data and corresponding embedding vector
- **Output**: True if successful, False otherwise
- **Errors**: May raise exception if Qdrant is unavailable