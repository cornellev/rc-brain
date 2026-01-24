# rc-brain
One repo to rule them all

## Installation (local)
`git clone --recurse-submodules https://github.com/cornellev/rc-brain.git`  
`cd rc-brain && sudo ./scripts/install_extra.sh`

## Development

### Local
Run `./scripts/deploy/arduino_setup.sh` the first time arduino is setup.
Run `./scripts/deploy/arduino_deploy.sh` to compile and flash code to the arduino.

#### Formatting
If you're not using the DevContainer, please make sure to install `clang-format`, ideally version 19! This will help keep the formatting of our C++ code formatted consistently. Python code is formatted by `black`. The DevContainer will install the `Black` extension for you, which ships with `black`, but otherwise you'll need to install it in VSCode or whatever other IDE you're using.

### Docker (DevContainer)
Everything should pretty much work out of the box, except...
> [!WARNING]  
> You must have the Docker buildx plugin installed to build the image correctly. You may get very confusing failures!
> On Ubuntu, you can install the `docker-buildx-plugin` package.

### Deployment

#### Regular deployment
If you're just developing, building Docker images will probably take too long. You can deploy normally just by running `scripts/deploy.sh`. It will drop you into an interactive session at the end so you can launch whatever you want to.

#### Docker deployment
Use `docker build . --file Dockerfile.deploy -t rc-brain-deploy:latest` to build the deploy container.
Then run either `docker run --rm rc-brain-deploy launch` or `docker run --rm rc-brain-deploy teleop` to launch.

> [!WARNING]  
> You must have the Docker buildx plugin installed to build the image correctly. You may get very confusing failures!
> On Ubuntu, you can install the `docker-buildx-plugin` package.

Our Docker deployment pipeline works by pushing Docker images to a local registry and pulling them down on the mini cars. This avoids both long build times on the mini cars and the long time it takes to transfer whole images (as we can take advantage of Docker's cache).

You'll have to do a bit of setup to get your computer to use the local registry. Note that you must be connected to `cev-router`.

Add the following configuration to `/etc/docker/daemon.json`:
```json
{
    "insecure-registries" : [ "mini-server:5000" ]
}
```

Then, restart the docker daemon (with `systemctl restart docker` on linux).

If you have errors regarding buildx or buildkit, please make sure you have Docker BuildX installed.

If there are still errors regarding some "Exec format error", then run
`docker run --privileged --rm tonistiigi/binfmt --install all`
which will install qemu for cross-platform compilation.

#### Remote Connection
To send and receive data over ros while on the same network, simply run
```
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0  
```
on all devices.