# Motivation

This repository includes the virtual environment for the The Construct Course. It was required due to our low speed internet connection in the lab.

# Installation

Requirements:
 
- Install docker engine
- Instal the correct cuda throught the NVIDIA Container Toolkit

## Installation Tips:

On my ubuntu 22.04 machine I had a problem with the initialization of the docker engine and the cuda toolkit. So I had a to disable Docker from starting automatically on booting.

    sudo systemctl disable docker

And then, when I want to use the docker I have to start manually. For that just add the following command to the .bashrc file:

    start-docker() {
        sudo systemctl start docker
    }

And then just call the function:

    start-docker

It will start the docker engine with the nvidia container toolkit.

## Building the image

To build the image run the following command:

    docker build -t turtlebot_lab .


# Usage

## Running the container

First of all type the following command to give to the container access to the host monitor:

    xhost +local:

To run the container run the following command:

    docker-compose up 

If you are using ubuntu 24.04 you may need to run the following command instead:

    docker compose up 

This will run the container and show to you the simulation.

Now you can interact with the system opening new terminals and running:

    docker exec -it turtlebot_lab /bin/bash



