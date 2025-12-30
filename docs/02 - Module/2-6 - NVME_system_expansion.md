# NVME system expansion

## Expanding NVMe System Storage

Because a running system's storage cannot be directly expanded, you must first remove the NVMe solid-state drive (SSD), place it into an external SSD enclosure, and then connect it to a computer (or virtual machine) to perform the necessary operations.

### 1. Open GParted

Open GParted on your virtual machine.

![](/img/docs/2-6/1642747459539245.png)

If GParted is not installed, use the following command to install it:

`sudo apt install gparted`

### 2. Select the NVMe Drive and Check Partition Information

Identify and select the correct NVMe drive (e.g., `/dev/nvme0n1`), then review its partition information. The primary partition, often labeled "APP", is typically `/dev/nvme0n1p1`. It is crucial to select the correct drive to avoid data loss.

You might observe that a portion of the "APP" partition appears gray. This is normal, and the goal is to convert this gray area to white. In GParted's visual representation:
*   Yellow indicates used space.
*   White indicates free/unused space.
*   Gray indicates unavailable space.

This gray area often occurs because a recovered or compressed system image requires its internal space to be re-checked before it can fully utilize the entire partition capacity.

### 3. Unmount the APP Partition

Right-click the "APP" partition and select **Unmount** to dismount it.

![](/img/docs/2-6/1642747463279541.png)

### 4. Check the APP Partition

With the "APP" partition still selected, right-click again and choose **Check**. Follow any on-screen prompts to complete this operation.

![](/img/docs/2-6/1642747466166076.png)

![](/img/docs/2-6/1642747469479370.png)

![](/img/docs/2-6/1642747472967346.png)

Upon successful completion, the gray area of the partition should now appear white, indicating available free space.

![](/img/docs/2-6/1642747478547149.png)

Once the partition check is complete, you can remove the solid-state drive from its external enclosure and reinstall it into your system.