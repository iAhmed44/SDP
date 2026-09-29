# Base Image Source: Debian Bookworm matches the native Raspberry Pi 5 OS environment
FROM python:3.11-slim-bookworm

# Set standard execution directory
WORKDIR /app

# Install system-level C-compilers and hardware libraries
# swig: Compiles the lgpio wrapper for direct Pi 5 GPIO pin control
# libgl1 & libglib2.0-0: Supplies graphics matrices required by OpenCV-headless
RUN apt-get update && apt-get install -y \
    build-essential \
    swig \
    libgl1 \
    libglib2.0-0 \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies first to leverage Docker layer caching
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the remaining subsystem files
COPY . /app/

# Execute the central orchestrator
CMD ["python3", "main_sensing.py"]