# Re-write system image

# Rewriting a System Image

This tutorial addresses a common issue where a Solid-State Drive (SSD), including NVMe types, has had a system image written to it, but the drive letter is subsequently not recognized by the computer. This lack of recognition prevents the drive from being re-imaged or re-burned with a new system image.

For example, Windows 7 systems typically cannot recognize partitions created by burning an Ubuntu system image. While Windows 10 might offer tools like SDformat for basic reformatting, a more comprehensive solution is often required.

This guide uses a USB flash drive as an example. The procedure is similar for SSDs when they are housed in an external enclosure and connected to your computer via a USB cable.

## Formatting the SSD

To prepare the drive for a new system image, it must first be reformatted to a file system that your operating system can recognize. We recommend using DiskGenius software to directly format the drive's partitions and create a single exFAT partition.

While this guide focuses on DiskGenius, other disk management utilities, including your computer's built-in disk management tools, can also perform similar operations.

### Deleting Existing Partitions

The first step is to delete all existing partitions on the target drive.

**Important Warning:** You must meticulously select the correct disk corresponding to your SSD or USB drive. Selecting the wrong drive will result in irreversible data loss on that disk.

Download DiskGenius software from the following link:
[https://www.diskgenius.com/download.php](https://www.diskgenius.com/download.php)

![](/img/docs/2-5/image-20240118091203532.png)

![](/img/docs/2-5/image-20240118092321219.png)

![](/img/docs/2-5/image-20240118093520765.png)

After reviewing the proposed changes, click "Confirm" to apply them.

### Creating a New Partition

Once all previous partitions have been deleted, proceed to create a new, single partition on the drive.

![](/img/docs/2-5/image-20240118094014328.png)

![](/img/docs/2-5/image-20240118094055227.png)

![](/img/docs/2-5/image-20240118094055227-1.png)

After completing the operations outlined above, your USB drive (or SSD) should now display its full storage capacity within your computer's disk management tools.

Finally, you can proceed to re-write the desired system image by following the standard image burning instructions.