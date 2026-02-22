---
title: 02-Choose Your Setup Path
sidebar_position: 2
---

# 02-Choose Your Setup Path

Use this page before doing any flashing or environment installation.

## 1. Recommended Path for Most Beginners

If your board already boots into the factory desktop, use:

- [03-Installing Jetson Environment](./03-installing-jetson-environment.md)

This is the safest and fastest path.

## 2. Which Path Should You Use?

| Your Goal | Recommended Lesson | Risk Level |
|-----------|--------------------|------------|
| Keep factory image and add NVIDIA components | [03-Installing Jetson Environment](./03-installing-jetson-environment.md) | Low |
| Reflash clean official NVIDIA system | [02-Write Jetson Original System](./04-write-jetson-original-system.md) | Medium |
| Move to SUPER system workflow | [06-Write SUPER Original System](./06-write-super-original-system.md) | High |

## 3. Quick Decision Rules

- If you are new to Jetson: choose `03-Installing Jetson Environment`.
- If you need a pure clean system and understand recovery flashing: choose `02-Write Jetson Original System`.
- If you explicitly need SUPER workflow compatibility: choose `06-Write SUPER Original System`.

## 4. Important Safety Note

Flashing can overwrite your current system. Back up important data before following reflash tutorials.

