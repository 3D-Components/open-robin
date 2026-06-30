# syntax=docker/dockerfile:1
FROM python:3.12-slim

ENV POETRY_VERSION=2.1.4 \
    POETRY_HOME="/opt/poetry" \
    POETRY_VIRTUALENVS_CREATE=false \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    POETRY_REQUESTS_TIMEOUT=600

RUN apt-get update && apt-get install -y --no-install-recommends \
      curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://install.python-poetry.org | python3 - --version $POETRY_VERSION
ENV PATH="$POETRY_HOME/bin:$PATH"

# Work at /app (not /robin)
WORKDIR /app

# Copy manifests from repo root (build context is repo root)
COPY pyproject.toml poetry.lock* ./

# Ensure lock is in sync, then install only runtime deps
RUN poetry install --only main --no-interaction --no-ansi --no-root

#  RUN poetry lock --no-interaction --no-ansi \
#  && poetry install --only main --no-interaction --no-ansi

# Copy your code (the package folder + any needed top-level files)
COPY robin ./robin
COPY robin/dashboard.html ./dashboard.html

# Start the alert processing service
# Use the serve command to start the FastAPI server
CMD ["python", "-m", "robin", "serve"]
