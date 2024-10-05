# rc-brain
One repo to rule them all


## Developing

### Local Development

#### Formatting
If you're not using the DevContainer, please make sure to install `clang-format`, ideally version 19! This will help keep the formatting of our C++ code formatted consistently. Python code is formatted by `black`. The DevContainer will install the `Black` extension for you, which ships with `black`, but otherwise you'll need to install it in VSCode or whatever other IDE you're using.

### Deploying
Our deployment pipeline works by pushing Docker images to a local registry and pulling them down on the mini cars. This avoids both long build times on the mini cars and the long time it takes to transfer whole images (as we can take advantage of Docker's cache).

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

## Packages

### teleop
The joystick is connected to `/dev/input/event0` by default. To be able to access it, the user launching ros must have the `input` group.
