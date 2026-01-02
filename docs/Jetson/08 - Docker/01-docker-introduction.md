---
sidebar_position: 1
title: Docker Introduction
---

# Docker Introduction

Docker is an open-source platform for developing, deploying, and running
applications. Through container technology, developers can create
**consistent**, **portable**, and **scalable** application environments,
improving development efficiency and application reliability.

::: note
In some regions, pulling Docker images from public registries may be
restricted due to network limitations. In such cases, the examples in
this tutorial are intended for demonstration and learning purposes.
:::

------------------------------------------------------------------------

## 1. Containers

A **Docker container** is a lightweight, independent, executable
software package that contains everything needed to run an application,
including:

-   Application code\
-   Runtime\
-   System tools\
-   System libraries\
-   Configuration settings

### Container Features

  -------------------------------------------------------------------------
  Feature        Description
  -------------- ----------------------------------------------------------
  Lightweight    Containers share the kernel of the host operating system

  Independence   Containers are isolated with their own filesystem,
                 processes, and networking

  Portability    Containers can run on any platform that supports Docker
  -------------------------------------------------------------------------

::: warning
Docker containers are built on top of the host operating system and CPU
architecture. Containers built for one architecture may not run on
another.
:::

------------------------------------------------------------------------

## 2. Images

A **Docker image** is a **read-only template** used to create
containers.

### Image Features

  -----------------------------------------------------------------------
  Feature         Description
  --------------- -------------------------------------------------------
  Immutability    Images are read-only once created

  Layered         Images are composed of multiple layers representing
  Structure       build steps
  -----------------------------------------------------------------------

::: note
If you modify data inside a running container, those changes are **not**
saved back to the image unless you explicitly create a new image from
the container.
:::

------------------------------------------------------------------------

## 3. Docker Engine

The **Docker Engine** is a client--server application consisting of:

-   Docker Daemon\
-   REST API\
-   Command Line Interface (CLI)

------------------------------------------------------------------------

### 3.1 Docker Daemon

The **Docker Daemon** is responsible for:

-   Building images\
-   Running containers\
-   Managing container lifecycle

------------------------------------------------------------------------

### 3.2 Docker CLI

The **Docker CLI** provides command-line tools to:

-   Build images\
-   Run containers\
-   Manage Docker resources

------------------------------------------------------------------------

## 4. Docker Hub

**Docker Hub** is a public cloud registry used to store and distribute
Docker images.

------------------------------------------------------------------------

## 5. Main Advantages of Docker

  -------------------------------------------------------------------------
  Advantage     Description
  ------------- -----------------------------------------------------------
  Consistency   Same environment across development and production

  Isolation     Improved security and stability

  Portability   Easy migration across platforms

  Efficiency    Lighter than virtual machines
  -------------------------------------------------------------------------

------------------------------------------------------------------------

Maintained by **HemiHex**.
