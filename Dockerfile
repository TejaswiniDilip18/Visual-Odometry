FROM ubuntu:20.04

# Prevent interactive prompts during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/* 

# Set the working dir
WORKDIR /app

# Copy the code into container
COPY . /app

# Create a build directory and navigate to it
RUN mkdir -p build && cd build && \
    cmake .. && make -j$(nproc)

WORKDIR /app/build

# Run the compiled executable
CMD [ "./vo" ]