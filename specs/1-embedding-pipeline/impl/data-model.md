# Data Model: Embedding Pipeline

## Document Entity

**Structure**:
```python
{
    "url": "string",           # Source URL of the document
    "title": "string",         # Title extracted from the page
    "content": "string",       # Clean extracted text content
    "created_at": "datetime",  # Timestamp when document was processed
    "word_count": "integer",   # Number of words in the content
    "checksum": "string"       # Hash of content to detect changes
}
```

## Embedding Entity

**Structure**:
```python
{
    "document_id": "string",   # Reference to source document
    "chunk_id": "string",      # Unique identifier for this chunk
    "chunk_text": "string",    # The text content of this chunk
    "embedding": "list[float]", # Vector representation from Cohere
    "chunk_size": "integer",   # Number of tokens/characters in chunk
    "chunk_index": "integer",  # Position of chunk in original document
    "metadata": {              # Additional metadata for retrieval
        "url": "string",
        "title": "string",
        "source_chunk": "string"
    }
}
```

## ProcessingJob Entity

**Structure**:
```python
{
    "job_id": "string",        # Unique identifier for the processing job
    "urls": ["string"],        # List of URLs to process
    "status": "string",        # Status: 'pending', 'processing', 'completed', 'failed'
    "created_at": "datetime",  # When the job was created
    "completed_at": "datetime", # When the job was completed (if applicable)
    "processed_count": "integer", # Number of URLs processed
    "failed_count": "integer", # Number of URLs that failed
    "error_details": ["string"] # List of error messages for failed URLs
}
```

## Qdrant Vector Structure

**Payload**:
```python
{
    "chunk_id": "string",      # Unique identifier for this chunk
    "document_url": "string",  # Source URL of the document
    "document_title": "string", # Title of the source document
    "chunk_text": "string",    # The text content of this chunk
    "chunk_index": "integer",  # Position of chunk in original document
    "created_at": "datetime",  # When this chunk was created
    "source_chunk": "string"   # Original chunk identifier
}
```

**Vector Configuration**:
- Vector size: 1024 (Cohere multilingual v3 model)
- Distance metric: Cosine
- Collection name: "rag-embadding"

## Text Chunking Parameters

**Default Values**:
- Chunk size: 512 tokens (approximately 400-500 words)
- Chunk overlap: 50 tokens (approximately 40-50 words)
- Minimum chunk size: 100 tokens
- Separator: Natural sentence boundaries where possible

## Metadata Schema

**For Qdrant Storage**:
- document_url: Source URL for the content
- document_title: Title of the source document
- chunk_index: Position in the original document
- created_at: Timestamp of when the chunk was processed
- word_count: Number of words in the chunk
- checksum: Hash of the chunk content for deduplication