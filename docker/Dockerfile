# Use Ubuntu 22.04 as the base image for compatibility with JMAVSim
FROM ubuntu:22.04

# Set non-interactive frontend to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies: OpenJDK, Maven, Ant, Git, and X11 utilities
RUN apt-get update && apt-get install -y \
    openjdk-11-jdk \
    maven \
    ant \
    git \
    x11-apps \
    libxrender1 \
    libxtst6 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*

# Clone JMAVSim repository and initialize submodules
WORKDIR /jmavsim
COPY EVERYTHING FROM THIS JMAVSIM repositor


# Build JMAVSim using ant to create the standalone JAR
RUN cd src &&  ant create_run_jar copy_res

# Expose port 4560 for JMAVSim (used for SITL communication)
EXPOSE 4560

WORKDIR  /jmavsim/src/out/production
# Set the entrypoint to run the script that handles JMAVSim execution
ENTRYPOINT ["java", "-Djava.ext.dirs=", "-jar", "jmavsim_run.jar"]