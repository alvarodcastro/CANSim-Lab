FROM python:3.11-slim

RUN apt-get update && apt-get install -y \
    iproute2 \
    can-utils \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip install python-can

WORKDIR /app
COPY ecu_simulator.py .
COPY ecu_definitions.py .

# The container needs NET_ADMIN to use CAN sockets
# and must run with host network or a macvlan to access vcan0

CMD ["python", "-u", "ecu_simulator.py"]
