FROM python:3.12-slim

WORKDIR /app

RUN apt-get update && \
    apt-get install -y --no-install-recommends libgl1 libglib2.0-0 && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
    viser \
    trimesh \
    yourdfpy \
    websockets \
    numpy \
    scipy

COPY robin-ui/services/viser_server.py /app/viser_server.py
COPY robin-ui/frontend/public/assets/robots/robot_a /app/assets/robots/robot_a
COPY robin-ui/frontend/public/assets/motion /app/assets/motion

ENV VISER_ASSETS_ROOT=/app/assets

EXPOSE 8081 8082

CMD ["python", "viser_server.py"]
