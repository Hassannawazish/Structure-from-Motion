# Use Python base image
FROM python:3.10-slim

# Set work directory in container
WORKDIR /app

# Copy requirements and install dependencies
COPY Requirements ./Requirements
RUN pip install --upgrade pip && pip install -r Requirements

# Copy entire project
COPY . .

# Set the default command to run your script
CMD ["python", "src/main.py"]
