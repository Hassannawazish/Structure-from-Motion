# Use Python base image
FROM python:3.10-slim

# Install OpenCV dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app/src

# Copy requirements and install Python dependencies
COPY requirement.txt /app/requirement.txt
RUN pip install --upgrade pip && pip install -r /app/requirement.txt

# ✅ Only copy src (which already contains Dataset, Matches, etc.)
COPY src /app/src

# Run your main script
CMD ["python", "main.py"]
