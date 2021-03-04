# drone-tracking-toolkits
This repository is for the IPA project [Measuring Drone Trajectory using Total Station with Visual Tracking](https://sirop.org/app/f76aed5f-95af-43db-a302-47ae994b03be) at ETH IGP. 

Student: Yue Pan

Supervisor: Dr. Cenek Albl, et al.

---

[**[Presentation slide]**]([](https://github.com/YuePanEdward/drone-tracking-toolkits/blob/main/doc/IPA_presentation-YuePan-20201217.pdf)) [**[Report]**](https://ethz.ch/content/dam/ethz/special-interest/baug/igp/photogrammetry-remote-sensing-dam/documents/pdf/Student_Theses/IPA_YuePan.pdf)

----

### Abstract

In this project, we managed to construct a **visual drone tracking and positioning dataset** collected by a multi-sensor system, including a **total station**, on-board sensor kits, and an ad-hoc **network of cameras**. 

By leveraging high accuracy total station measurements and sensor fusion techniques such as the extended Kalman filter, the absolute positioning accuracy for the drone’s body center in the local frame can be better than **one centimeter**. By employing a radio-synchronized network of audio triggers, we can recognize the triggering pattern from the audio signal of each video, thus accomplishing the synchronization among the videos with sub-frame accuracy. The overall synchronization time delay of the system is about **10 milliseconds** after aligning all the measurements’ timestamp to the referenced PC time.

### Dataset acquired using the toolkit

*Drone visual tracking and trajectory reconstruction dataset* [**[link to the dataset]**](https://github.com/CenekAlbl/drone-tracking-datasets)

![alt text](doc/imgs/dataset_overview.jpg)

### Frames overview
![alt text](doc/imgs/pose_overview.jpg)

### Synchronization overview
![alt text](doc/imgs/sync_overview.jpg)

-----

#### Useful links:

[The previous dataset](https://github.com/CenekAlbl/drone-tracking-datasets)

[Codes for the visual trajectory drone reconstruction framework](https://github.com/CenekAlbl/mvus)

[Reconstruction of 3D flight trajectories from ad-hoc camera networks (IROS '20)](https://arxiv.org/abs/2003.04784)

#### Acknowledgement:

Special thanks to Prof. Dr. Konrad Schindler, Dr. Cenek Albl, Dr. Jemil Butt, Andreas Baumann-Ouyang, Alexander Wolf, Thomas Posur, Tom Manu, Usvyatsov Mikhail and Mudathir Awadaljeed from [ETH Zurich Institute of Geodesy and Photogrammetry](https://igp.ethz.ch/) for the supervision, advice and help during the project.