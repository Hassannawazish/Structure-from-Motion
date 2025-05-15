# Use Python base image
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Copy actual file (requirement.txt)
COPY requirement.txt ./requirement.txt
RUN pip install --upgrade pip && pip install -r requirement.txt

# Copy everything else
COPY . .

# Set the default command
CMD ["python", "src/main.py"]
