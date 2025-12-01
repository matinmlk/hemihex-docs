# SSD Writing System

1. **Remove the NVMe (solid-state drive) from the Jetson Orin Nano**, insert it into an NVMe/SSD enclosure, and tighten the screws.

   ![](/img/docs/2-3/2023050900001.png)

2. Connect the SSD enclosure to your computer using USB.  
   If the NVMe already contains data, **please format it first**, then continue.

   **2.5 SSD Rewrite NVMe System**

3. **Open the Win32DiskImager tool.**

   ![](/img/docs/2-3/2023050900002.png)

4. If your SSD has no partition and the tool cannot detect a drive letter, you must partition it.  
   Refer to the instructions in **2.5 SSD Rewrite NVMe System**.

5. If the SSD is already partitioned:  
   - Select the image file  
   - Select the SSD drive letter  
   - Click **Write**, as shown below:

   ![](/img/docs/2-3/2023050900003.png)

6. Wait until the image writing process is finished.  
   Then insert the NVMe back into the Jetson Orin Nano and power it on.  
   After a short time, the system will boot successfully.

   ![](/img/docs/2-3/2023050900004.png)
