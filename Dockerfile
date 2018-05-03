# Use an official Rust nightly compiler as a parent image
FROM rustlang/rust:nightly

# Set the working directory to /app
WORKDIR /app

# Copy the current directory contents into the container at /app
ADD . /app

# Install any needed dependencies and compile
RUN cargo build --release --all

# Make port 80 available to the world outside this container
EXPOSE 80

# Define environment variable
ENV ROCKET_ENV prod

WORKDIR /app/server

# Run server when the container launches
CMD ["cargo", "run", "--release", "--bin", "bmw_routing_server", "--", "/data"]