---
sidebar_position: 3
title: Docker Commands
---

# Docker Commands

Docker Engine includes the Docker CLI, which provides command-line tools
for interacting with the Docker daemon. This document introduces
commonly used Docker commands.

------------------------------------------------------------------------

## 1. View Detailed Information

``` bash
docker info
```

![Docker Info Output](/img/docs/jetson/8-docker/8-3/image-20250105134326862.png)

------------------------------------------------------------------------

## 2. View the Version Number

``` bash
docker --version
```

![Docker Version](/img/docs/jetson/8-docker/8-3/image-20250105134400721.png)

------------------------------------------------------------------------

## 3. Pull an Image

Pull the latest version of an image:

``` bash
docker pull <image_name>
```

Example:

``` bash
docker pull hello-world
```

Pull a specific tag:

``` bash
docker pull <image_name>:<tag>
```

Example:

``` bash
docker pull ubuntu:18.04
```

![Docker Pull](/img/docs/jetson/8-docker/8-3/image-20250105141705494.png)

------------------------------------------------------------------------

## 4. Run an Image

If the image does not exist locally, Docker will pull it automatically.

``` bash
docker run <image_name>
```

Example:

``` bash
docker run hello-world
```

Run and exit immediately:

``` bash
docker run ubuntu:18.04
```

Run in interactive mode:

``` bash
docker run -it ubuntu:18.04 /bin/bash
```

Exit the container with:

``` bash
exit
```

![Docker Run](/img/docs/jetson/8-docker/8-3/image-20250105142432302.png)

------------------------------------------------------------------------

### 4.1 View Running Containers

``` bash
docker ps
```

------------------------------------------------------------------------

### 4.2 View All Containers (Running + Stopped)

``` bash
docker ps -a
```

![Docker PS](/img/docs/jetson/8-docker/8-3/image-20250105142517865.png)

------------------------------------------------------------------------

## 5. Clean Up Containers

Remove all stopped containers:

``` bash
docker container prune
```

![Docker Prune](/img/docs/jetson/8-docker/8-3/image-20250105142635954.png)

------------------------------------------------------------------------

## 6. View Local Images

``` bash
docker images
```

![Docker Images](/img/docs/jetson/8-docker/8-3/image-20250105142659185.png)

------------------------------------------------------------------------

## 7. Delete Images

:::warning
The image must not be used by any running container before deletion.
:::

Delete an image:

``` bash
docker rmi <image_name>
```

Examples:

``` bash
docker rmi hello-world
docker rmi ubuntu:18.04
```

![Docker RMI](/img/docs/jetson/8-docker/8-3/image-20250105143042342.png)

------------------------------------------------------------------------

## 8. Commit a Container as an Image

Run a container interactively:

``` bash
docker run -it ubuntu:18.04 /bin/bash
```

Commit the container as a new image:

``` bash
docker commit <CONTAINER_ID> <image_name>:<tag>
```

:::note
Replace `<CONTAINER_ID>` with the actual container ID from
`docker ps -a`.
:::

------------------------------------------------------------------------

## 9. Stop a Container

``` bash
docker stop <container_id>
```

------------------------------------------------------------------------

## 10. Access a Running Container from Multiple Terminals

``` bash
docker exec -it <container_id> /bin/bash
```

------------------------------------------------------------------------

Maintained by **HemiHex**.
