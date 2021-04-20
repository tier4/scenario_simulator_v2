# Run with docker

## Install docker

Please follow instructions below.

<iframe
    src="https://hatenablog-parts.com/embed?url=https%3A%2F%2Fdocs.docker.com%2Fengine%2Finstall%2F" 
    title="Install Docker Engine" 
    class="embed-card embed-webcard"
    scrolling="no"
    frameborder="0"
    style="display: block; width: 100%; height: 155px; max-width: 500px; margin: 10px 0px;">
</iframe>

## Install nvidia-docker2 (optional)

If you have nvidia GPU, in your machine, you have to install nvidia-driver and install nvidia-docker2

### Ubuntu

In order to install vidia-docker2 in ubuntu, please type commands below.

```
curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
sudo apt-get update
```
