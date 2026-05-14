FROM python:3.11-slim

RUN apt-get update && apt-get install -y \
    iproute2 \
    can-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip install flask flask-socketio python-can eventlet

WORKDIR /app
COPY app.py .
COPY templates/ templates/
COPY static/ static/

EXPOSE 5000

CMD ["python", "-u", "app.py"]
