FROM espressif/idf:release-v5.1

RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    vim

# Add Python ML dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages -r /tmp/requirements.txt