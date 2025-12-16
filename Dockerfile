# Use Python 3.11 slim image
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy backend code
COPY backend/ ./backend/

# Copy src directory (for .env file path resolution)
COPY src/ ./src/

# Expose port (Railway provides PORT env variable)
EXPOSE 8000

# Start command - use shell form to allow PORT env variable expansion
CMD uvicorn backend.app.main:app --host 0.0.0.0 --port ${PORT:-8000}
