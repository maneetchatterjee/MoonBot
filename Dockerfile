FROM python:3.11-slim

WORKDIR /workspace
ENV MUJOCO_GL=egl
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg libegl1 && rm -rf /var/lib/apt/lists/*
COPY pyproject.toml README.md ./
COPY moonbot ./moonbot
COPY assets ./assets
COPY configs ./configs
COPY scripts ./scripts
RUN pip install --no-cache-dir -e ".[analysis,control,learning,test]"

CMD ["python", "-m", "moonbot.cli", "validate-model"]
