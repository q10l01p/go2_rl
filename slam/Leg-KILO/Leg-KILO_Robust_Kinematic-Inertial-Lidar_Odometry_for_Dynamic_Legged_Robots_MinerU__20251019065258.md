# Leg-KILO: Robust Kinematic-Inertial-Lidar Odometry for Dynamic Legged Robots

Guangjun Ou  $①$ , Dong Li  $①$ , and Hanmin Li  $①$

Abstract—This letter presents a robust multi-sensor fusion framework, Leg-KILO (Kinematic-Inertial-Lidar Odometry). When lidar-based SLAM is applied to legged robots, high-dynamic motion (e.g., trot gait) introduces frequent foot impacts, leading to IMU degradation and lidar motion distortion. Direct use of IMU measurements can cause significant drift, especially in the z-axis direction. To address these limitations, we tightly couple leg odometry, lidar odometry, and loop closure module based on graph optimization. For leg odometry, we propose a kinematic-inertial odometry using an on-manifold error-state Kalman filter, which incorporates the constraints from our proposed contact height detection to reduce height fluctuations. For lidar odometry, we present an adaptive scan slicing and splicing method to alleviate the effects of high-dynamic motion. We further propose a robot-centric incremental mapping system that enhances map maintenance efficiency. Extensive experiments are conducted in both indoor and outdoor environments, showing that Leg-KILO has lower drift performance compared to other state-of-the-art lidar-based methods, especially during high-dynamic motion. To benefit the legged robot community, a lidar-inertial dataset containing leg kinematic data and the code are released.

Index Terms—Legged robots, localization, SLAM, sensor fusion.

# I. INTRODUCTION

REAL-TIME state estimation is a crucial component in the navigation and control of legged robots. While robots operating in GPS-denied or poor lighting conditions, Lidar-Inertial Odometry (LIO) is a popular solution. Lidar can directly obtain dense environmental information with depth, while the integration of high-frequency IMU measurements can enhance the system's dynamic response. However, due to the limited sampling frequency and the measurement range, the accelerometer often degrades in the presence of large impacts or high-frequency vibrations [1], [2], which might affect the accuracy of pure LIO system. Fig. 3 shows the results of two subsystems of LIO-SAM [3], IMU odometry and LiDAR odometry, on flat ground. Due to the dynamic motion of the legged robot, the IMU odometry accumulates significant errors in a short time.

Manuscript received 22 March 2024; accepted 19 July 2024. Date of publication 8 August 2024; date of current version 16 August 2024. This article was recommended for publication by Associate Editor K. Akbari H. and Editor Abderrahmane Kheddar upon evaluation of the reviewers' comments. This work was supported by Guangdong Basic and Applied Basic Research Foundation under Grant 2021A1515011867. (Corresponding author: Dong Li.)

The authors are with the School of Automation, Guangdong University of Technology, Guangzhou 510006, China (e-mail: ouguangjun98@gmail.com; dong.li@gdut.edu.cn; leehanmincloud@gmail.com).

This letter has supplementary downloadable material available at https://doi.org/10.1109/LRA.2024.3440730, provided by the authors.

Digital Object Identifier 10.1109/LRA.2024.3440730

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/8a04e603bafb61699e5f9fc165ec9c6412443b69d4a3a8b5cc0fa0634e72d9a4.jpg)  
Fig. 1. Left: The map is being built while the legged robot operating across the corridor. Right: Frames and sensors installed on the Unitree Go1. More details are shown in the video at https://youtu.be/6O74De5BLeQ.

Kim et al. [1] presented an adjustable neck system to absorb the impact in dynamic motion, showing the robustness of the estimation performance. Many previous works [4], [5], [6] utilized leg kinematic information to provide more constraints for legged robot state estimation. These kinematic-inertial methods still have a large drift under long-term operation. They may fail when a sudden situation occurs (e.g., foot slip) and cannot get enough observations of the yaw angle and the position [4], [7]. The introduction of exteroceptive sensors can effectively address the limitations. Currently many methods [8], [9], [10], [11], [12], [13], [14] that integrate exteroceptive sensors (e.g. lidar and camera) have been validated in various environments.

However, most of the above methods about legged robots have been tested at a low or moderate speed, and seldom investigated state estimation during high-dynamic motion. In this letter, our aim is to improve the performance of lidar-based SLAM in dynamic legged robots. We propose a tightly coupled Kinematic-Inertial-Lidar Odometry framework, Leg-KILO. The system architecture is shown in Fig. 2. Leg-KILO has the following contributions:

- We present a novel leg odometry using an on-manifold error-state Kalman filter (ESKF), tightly coupling leg kinematic and IMU. The height constrains from the contact height detection are fused to reduce the effect of foot impacts. To enhance global localization accuracy, we tightly couple leg odometry, lidar odometry, and loop closure module based on graph optimization.

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/40755e67cbdff0d5bc65f1e83a687f2f52215618196264914c17c015d599e69a.jpg)  
Fig. 2. System overview.

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/03a37038580af7fb95573571f35ee44ebfba8cc04c061038f67434d193e54673.jpg)  
Fig. 3. Comparison of body height's variation for the two subsystems of LIOSAM [3]–IMU odometry and lidar odometry, on sequence corridor. Due to the dynamic motion of the legged robot, the IMU odometry accumulates significant errors in a short time.

- To alleviate the effects of high-dynamic motion, we propose an Adaptive Scan Slice and Splice method to improve the input frequency based on the robot's motion velocity. We further present an incremental robot-centric map approach to enhance map maintenance efficiency, addressing potential heavy computation due to the increased input frequency.  
- The experimental result demonstrate that Leg-KILO achieves lower drift compared to other LIO methods. The dataset<sup>1</sup> and the code<sup>2</sup> are released for the benefit of the community.

# II. RELATE WORKS

Multi-sensor fusion is a commonly used solution for robot state estimation. For legged robots, some earlier methods [4], [5], [6] focused on the use of proprioceptive sensors, such as joint encoders, contact sensors, and IMU, for state estimation. Bloesch et al. [5] proposed an Unscented Kalman Filter (UKF) estimator that added the kinematic velocity to the filter observation. Bledt et al. [6] combines the above observation but utilizes a linear Kalman filter (KF) for estimation. In order to reduce the cumulative error, they also take the observation of contact height with respect to the global frame as a filter measurement, but they set this measurement manually because it cannot be obtained only by proprioceptive sensors. Kim et al. [15] proposed a state estimator capable of utilizing the Gauss-Newton method and handling errors during sudden situations (e.g., foot slip). Hartley et al. [16] uses contact preintegration to build constraints between keyframes, avoiding the excessive introduction of factor graph nodes.

TABLEI COMPARISON OF RELATED DATASETS ON LEGGED ROBOTS  

<table><tr><td>Dataset</td><td>Robot</td><td>Ground Truth</td><td>Speed [m/s] mean (max)</td><td>Camera</td><td>Lidar</td><td>IMU</td><td>Kin.</td><td>Open</td></tr><tr><td>VILENS [14]</td><td>ANYmal</td><td>LT/ICP</td><td>0.25 (-)</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td></td></tr><tr><td>Cerberus [20]</td><td>ANYmal</td><td>ICP</td><td>0.36 (0.7)</td><td>✓</td><td>✓</td><td>✓</td><td></td><td>✓</td></tr><tr><td>NeBula [21]</td><td>Spot</td><td>ICP</td><td>0.53 (1.2)</td><td>✓</td><td>✓</td><td>✓</td><td></td><td>✓</td></tr><tr><td>Leg-KILO</td><td>Go1</td><td>ICP/Slam/(start end)</td><td>0.80 (2.0)</td><td></td><td>✓</td><td>✓</td><td>✓</td><td>✓</td></tr></table>

LT: Laser Tracker (start end): Evaluate the drift when returning to the start point.

Although the kinematic-inertial methods are low-cost and efficient for state estimation, the absolute position and yaw angle cannot be directly observed [4], [7], leading to significant drift over long periods of movement. Many methods [8], [9], [10], [11], [12], [13], [16] that further integrate cameras have been validated for their robustness, outperforming kinematic-inertial methods.

Utilizing lidar for state estimation is currently a trending topic in robotics. Shan et al. further improved upon LOAM [17] and proposed a lidar-inertial factor graph framework, LIO-SAM [3]. Xu et al. [18] presented a filter-based lidar-inertial odometry, FAST-LIO2, which is famous for its low-latency. However, above state-of-the-art LIO methods might perform not well when applied to legged robots. This is because the high-frequency impacts of the robot's foot on the ground make accelerometer measurements unstable, resulting in significant drift along  $z$ -axis. POINT-LIO [19] is an extension of FASTLIO2 but uses the IMU as a filter measurement rather than an input, and does not rely too much on the IMU measurement. Our experiments also demonstrate its robustness during aggressive motion for legged robots. VILENS [14] comprehensively couples leg kinematics and IMU with lidar and camera to avoid the situation where the degradation of a single sensor leads to incorrect state estimation. VILENS uses graph factor to optimize visual and lidar features to get an accurate state estimation, but runs ICP at a low frequency (2Hz) due to the heavy computation.

For the lidar-based dataset on legged robots, VILENS [14] collected extensive data on different ANYmal robots, but is not publicly available. The datasets in [20] and [21] are both from the DARPA Subterranean Challenge, but lack raw data about leg kinematics. To verify our Leg-KILO and other lidar-based methods on dynamic legged robots, we conduct extensive data collection including encoders, contact sensors, lidar and IMU, and make our dataset publicly available. More information about our dataset and others on legged robots could be found in Table I.

# III. LEG ODOMETRY

# A. Filter State Definition

The notation involved in this letter and its explanations are shown in Table II. For leg odometry, we define the filter state  $\mathbf{x} \in \mathrm{SO}(3) \times \mathbb{R}^{33}$  as follows:

$$
\left[ ^ {W} \mathbf {R} ^ {W} \mathbf {p} ^ {W} \mathbf {v} ^ {W} \mathbf {c} _ {1} \dots^ {W} \mathbf {c} _ {4} ^ {W} \mathbf {g} \mathbf {b} _ {a} \mathbf {b} _ {\omega} \mathbf {a} \omega \right] (1)
$$

$^{1}$ https://github.com/ouguangjun/legkilo-dataset

$^{2}$ https://github.com/ouguangjun/Leg-KILO

<sup>3</sup>https://github.com/leggedrobotics/cerberus_darpa_subtDatasets

<sup>4</sup>https://github.com/NeBula-Autonomy/nebula-odometry-dataset

TABLE II NOTATIONS  

<table><tr><td>Notation</td><td>Explanation</td></tr><tr><td>⊕/ ⊕</td><td>The boxplus and boxminus operators on manifold</td></tr><tr><td>Exp(·) / Log(·)</td><td>The map between manifold and tangent vector</td></tr><tr><td>δ(·)</td><td>The error of the state (·)</td></tr><tr><td>(·)^</td><td>The skew-symmetric matrix</td></tr><tr><td>(·)/(·)</td><td>The prior and posterior of state (·)</td></tr></table>

where  ${}^{W}\mathbf{R} \in \mathrm{SO}(3),{}^{W}\mathbf{p} \in \mathbb{R}^{3},{}^{W}\mathbf{v} \in \mathbb{R}^{3}$  is the body rotation matrix, position and velocity in the world frame W (i.e., the initial body frame). For easy use of leg kinematics, we add the foot contact points  $[{}^{W}\mathbf{c}_{1},{}^{W}\mathbf{c}_{2},{}^{W}\mathbf{c}_{3},{}^{W}\mathbf{c}_{4}]$  in the world frame W to the filter state.  ${}^{W}\mathbf{g}$  is the unknown gravity vector in the world frame, which need to be initialization at first.  $\mathbf{b}_a$ ,  $\mathbf{b}_{\omega}$  are the IMU biases in the body frame. Erroneous measurements from the IMU can result in inaccurate state estimation when IMU is used as input. Given these limitations, we are inspired by [19] to treat the IMU as a model observation, and add the acceleration a and the angular velocity  $\omega$  in the body frame to the filter state.

# B. System Kinematics

According to the kinematics of the system, we define the continuous time model of the system at time  $t$  as follows:

$$
{ } ^ { W } \dot { \mathbf { R } } = { } ^ { W } \mathbf { R } \boldsymbol { \omega } ^ { \wedge } , \quad { } ^ { W } \dot { \mathbf { p } } = { } ^ { W } \mathbf { v } ,
$$

$$
{ } ^ { W } \dot { \mathbf { v } } = { } ^ { W } \mathbf { R a } + { } ^ { W } \mathbf { g } , \quad { } ^ { W } \dot { \mathbf { g } } = \mathbf { 0 } ,
$$

$$
\dot {\mathbf {b}} _ {a} = \mathbf {n} _ {a}, \quad \dot {\mathbf {b}} _ {\omega} = \mathbf {n} _ {\omega},
$$

$$
\dot {\mathbf {a}} = \mathbf {w} _ {a}, \qquad \dot {\boldsymbol {\omega}} = \mathbf {w} _ {\omega}
$$

$$
^ {W} \dot {\mathbf {c}} _ {i} = \mathbf {w} _ {c _ {i}} \quad i \in \{1, \dots , 4 \} \tag {2}
$$

where  $\mathbf{n}_a$ ,  $\mathbf{n}_{\omega}$  represents the Gaussian noise of IMU biases. As described in [19], the random walk of the robot's acceleration and angular velocity can be driven by Gaussian noise  $\mathbf{w}_a$  and  $\mathbf{w}_{\omega}$ , respectively, due to the smoothness of the robot movement. Due to the presence of foot slippage, the random foot position walk can also be described by white noise  $\mathbf{w}_c$ , which will be magnified infinitely while the leg swings and reset while the foot makes contact with the ground.

Then, the continuous model (2) is discrete as below:

$$
\mathbf {x} _ {k + 1} = \mathbf {x} _ {k} \boxplus \mathbf {f} \left(\mathbf {x} _ {k}, \mathbf {w} _ {k}\right) \tag {3}
$$

where  $k$  is the index of the filter process and the noise vector  $\mathbf{w}$  is defined as  $[\mathbf{w}_{c1}\dots \mathbf{w}_{c4}\mathbf{n}_a\mathbf{n}_\omega \mathbf{w}_a\mathbf{w}_\omega ]$ .  $\mathbf{f}(\mathbf{x},\mathbf{w})$  is the kinematics function:

$$
\left[ \omega^ {W} \mathbf {v} ^ {W} \mathbf {R a} + ^ {W} \mathbf {g} \mathbf {w} _ {c _ {1}} \dots \mathbf {w} _ {c _ {4}} \mathbf {n} _ {a} \mathbf {n} _ {\omega} \mathbf {0} \mathbf {w} _ {a} \mathbf {w} _ {\omega} \right] (4)
$$

# C. State Prediction

Suppose the predict process goes from the last time  $k$  to the current time  $k + 1$ , the prior state  $\hat{\mathbf{x}}_{k + 1}$  can be propagated through the formula (3) without considering the noise  $\mathbf{w}$ :

$$
\hat {\mathbf {x}} _ {k + 1} = \bar {\mathbf {x}} _ {k} \boxplus \mathbf {f} (\bar {\mathbf {x}} _ {k}, \mathbf {0}) \Delta t. \tag {5}
$$

The error state between the ground-truth state and the prior state can be linearized as:

$$
\begin{array}{l} \delta \mathbf {x} _ {k + 1} = \mathbf {x} _ {k + 1} \boxminus \hat {\mathbf {x}} _ {k + 1} \\ \simeq \mathbf {F} _ {\delta \mathbf {x}} \delta \mathbf {x} _ {k} + \mathbf {F} _ {\mathbf {w}} \mathbf {w} _ {k} \tag {6} \\ \end{array}
$$

where  $\mathbf{F}_{\delta \mathbf{x}}$  and  $\mathbf{F}_{\mathbf{w}}$  are Jacobian matrices of  $\mathbf{x} \boxminus \hat{\mathbf{x}}$  with respect to  $\delta \mathbf{x}$  and  $\mathbf{w}$ , respectively. Since both  $\delta \mathbf{x}$  and  $\mathbf{w}$  are Gaussian distributions, the prior covariance can be linearized as follows:

$$
\hat {\mathbf {P}} _ {k + 1} = \mathbf {F} _ {\delta \mathbf {x}} \bar {\mathbf {P}} _ {k} \mathbf {F} _ {\delta \mathbf {x}} ^ {\mathrm {T}} + \mathbf {F} _ {\mathbf {w}} \mathbf {Q} _ {k} \mathbf {F} _ {\mathbf {w}} ^ {\mathrm {T}} \tag {7}
$$

where  $\bar{\mathbf{P}}$  is the posterior covariance matrix and  $\mathbf{Q}$  is the covariance matrix of  $\mathbf{w}$ .

# D. Measurements

1) Foot Velocity Measurement: The foot velocity  $W \mathbf{v}_{ci}$  in the world frame can be computed as [5]:

$$
{ } ^ { W } \mathbf { v } _ { c i } = { } ^ { W } \mathbf { v } + { } ^ { W } \mathbf { R } \left( \boldsymbol { \omega } ^ { \wedge B } \mathbf { c } _ { i } + { } ^ { B } \mathbf { v } _ { c i } \right) .  ( 8 )
$$

where  ${}^B\mathbf{c}_i = \mathbf{g}_c(\pmb{\alpha}_i)$  and  $\mathbf{g}_c(\pmb{\alpha}_i)$  is the leg  $i$  forward kinematics function through the joint angle measurement  $\alpha_{i}$ . Assuming that there is no relative slip when the i'th foot makes contact with the ground, the foot velocity measurement can be obtained, which is zero in the ground-truth state:

$$
\mathbf {0} = \mathbf {h} _ {F V} (\mathbf {x}, \mathbf {n} _ {F V}) = ^ {W} \mathbf {v} + ^ {W} \mathbf {R} \left(\omega^ {\wedge B} \mathbf {c} _ {i} + ^ {B} \mathbf {v} _ {c i}\right) + \mathbf {n} _ {c} \tag {9}
$$

We define the noise caused by imprecise modeling as  $\mathbf{n}_c$ .

2) Foot Position Measurement: Given the rotation matrix, body position, and foot position in the world frame, the foot position in the body frame can be calculated as follows:

$$
{ } ^ { B } \mathbf { c } _ { i } = \mathbf { h } _ { F P } ( \mathbf { x } , \mathbf { n } _ { F P } ) = ^ { W } \mathbf { R } ^ { \mathrm { T } } \left( ^ { W } \mathbf { c } _ { i } - ^ { W } \mathbf { p } \right) + \mathbf { n } _ { F P } \tag {10}
$$

where  $\mathbf{n}_{FP}$  is the Gaussian noise included in the measurement model.

3) Contact Height Measurement: The introduction of contact height measurement can help to reduce drift along the Z-axis of the robot [6]. This letter proposes a contact height detection, which involves measuring contact height by utilizing a robot-centric incremental map constructed from historical point clouds. When foot  $i$  makes contact with the ground at time  $t_n$ , the foot position  ${}^B\mathbf{c}_i = \mathbf{g}_c(\pmb{\alpha}_i)$  that can be transformed to the world frame as follows:

$$
{ } ^ { W } \hat { \mathbf { c } } _ { i } = \bar { \mathbf { T } } _ { f , m } \bar { \mathbf { T } } _ { l e g , m } ^ { - 1 } \hat { \mathbf { T } } _ { l e g , n } \mathbf { g } _ { c } ( \boldsymbol { \alpha } _ { i } ) \tag {11}
$$

where  $\bar{\mathbf{T}}_{f,m}$  represents the transformation optimized by the factor graph (see Section V) at the nearest past time  $t_m$  ( $t_m < t_n$ ), while  $\hat{\mathbf{T}}_{leg,m}$  and  $\hat{\mathbf{T}}_{leg,n}$  represent the transformation of the leg odometry at time  $t_m$  and  $t_n$ , respectively. The incorporation of the optimized state from factor graph optimization to predict foot position is aimed at mitigating the effects of long-term cumulative errors in leg odometry. We assume that the local ground around the foot  $i$  (within  $10\mathrm{cm}$  distance) is flat at the time of contact, and the foot  $i$  lies on a local small plane. This assumption is reasonable in a wider range of paved surfaces, such as cement paths, paved floors, and slopes. We first utilize the foot position  $^W\hat{\mathbf{c}}_i$  to search its ten corresponding points  $\mathcal{P}_c$

(within  $10\mathrm{cm}$  distance) through the robot-centric incremental map (see Section IV.B), then the mean  $\bar{\mathbf{q}}$  and the covariance  $\mathbf{A}$  of the corresponding points  $\mathcal{P}_c$  can be defined as follows:

$$
\begin{array}{l} \mathbf {A} = \frac {1}{N} \sum_ {k = 1} ^ {N} \left(\mathbf {q} _ {\mathbf {k}} - \bar {\mathbf {q}}\right) \left(\mathbf {q} _ {\mathbf {k}} - \bar {\mathbf {q}}\right) ^ {T} \\ \bar {\mathbf {q}} = \frac {1}{N} \sum_ {k = 1} ^ {N} \mathbf {q} _ {\mathbf {k}}, \quad \mathbf {q} _ {\mathbf {k}} \in \mathcal {P} _ {c}. \tag {12} \\ \end{array}
$$

According to the principles of Principal Component Analysis (PCA), the unit normal vector  $\mathbf{u}$  of a plane can be obtained by solving for the eigenvector corresponding to the smallest eigenvalue of the covariance matrix  $\mathbf{A}$ . Thus, a plane can be represented as  $\{\mathbf{u}^T (\mathbf{x} - \bar{\mathbf{q}}) = 0 | \mathbf{u}, \bar{\mathbf{q}} \in \mathbb{R}^3\}$ . To detect whether the foot is in contact with the plane, we define the roughness of the plane by the point-to plane residual:

$$
d = \frac {1}{N} \sum_ {k = 1} ^ {N} \mathbf {u} ^ {T} \left(\mathbf {q} _ {\mathbf {k}} - \bar {\mathbf {q}}\right), \quad \mathbf {q} _ {\mathbf {k}} \in \mathcal {P} _ {c}. \tag {13}
$$

If  $d$  is greater than a certain threshold  $(0.01\mathrm{m})$ , the plane is considered to have low confidence and is discarded. When the foot is in contact with the plane, the foot is assumed to be coincides with its projection perpendicular to the plane. The foot  $i$ 's projection point  $\mathbf{q}_{proj}$  can be expressed as:

$$
\mathbf {q} _ {p r o j} = ^ {W} \hat {\mathbf {c}} _ {i} - \mathbf {u} ^ {T} \left(^ {W} \hat {\mathbf {c}} _ {i} - \bar {\mathbf {q}}\right) \mathbf {u}. \tag {14}
$$

Then the contact height measurement can be defined as:

$$
\mathbf {e} _ {z} \mathbf {q} _ {\text {p r o j}} = \mathbf {h} _ {C H} (\mathbf {x}, \mathbf {n} _ {z}) = \mathbf {e} _ {z} ^ {W} \mathbf {c} _ {i} + \mathbf {n} _ {z} \tag {15}
$$

where  $\mathbf{e}_z = [0,0,1]$  and the contact height carries a Gaussian noise  $\mathbf{n}_z = \mathcal{N}(0,|d\mathbf{e}_z\mathbf{u}|)$ , which is correlated with the roughness of the plane.

4) IMU Measurement: When the IMU measurements are received, the measurement function of IMU can be expressed:

$$
\begin{array}{l} \tilde {\mathbf {a}} = \mathbf {h} _ {a} (\mathbf {x}, \mathbf {n} _ {a m}) = \mathbf {a} + \mathbf {b} _ {a} + \mathbf {n} _ {a m} \\ \tilde {\omega} = \mathbf {h} _ {\omega} (\mathbf {x}, \mathbf {n} _ {\omega m}) = \boldsymbol {\omega} + \mathbf {b} _ {\omega} + \mathbf {n} _ {\omega m} \tag {16} \\ \end{array}
$$

where  $\mathbf{n}_{am}$  and  $\mathbf{n}_{\omega m}$  are Gaussian noises included in the measurement model. Specifically, due to the impact and vibration on the accelerometer measurements, we first examine whether the measurements  $\tilde{\mathbf{a}}$  are within the range of measurement rated. If the measurements are saturated, we increase their measurement covariance to a large value.

# E. Residual and Update

When the actual measurement  $\mathbf{h}(\mathbf{x},\mathbf{n})$  and the predicted measurement  $\mathbf{h}(\hat{\mathbf{x}},\mathbf{0})$  are known, then the residual  $\mathbf{r}_{k + 1}$  at step  $k + 1$  can be linearized as follows:

$$
\begin{array}{l} \mathbf {r} _ {k + 1} = \mathbf {h} (\mathbf {x} _ {k + 1}, \mathbf {n} _ {k + 1}) - \mathbf {h} (\hat {\mathbf {x}} _ {k + 1}, \mathbf {0}) \\ = \mathbf {h} (\hat {\mathbf {x}} _ {k + 1} \boxplus \delta \mathbf {x} _ {k + 1}, \mathbf {n} _ {k + 1}) - \mathbf {h} (\hat {\mathbf {x}} _ {k + 1}, \mathbf {0}) \\ \simeq \mathbf {H} _ {k + 1} \delta \mathbf {x} _ {k + 1} + \mathbf {D} _ {k + 1} \mathbf {n} _ {k + 1} \tag {17} \\ \end{array}
$$

where  $\mathbf{H}$  and  $\mathbf{D}$  are the Jacobian matrix of residual  $\mathbf{r}$  with respect to  $\mathbf{x}$  and  $\mathbf{n}$ , respectively. Detailed Jacobian matrices for

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/9977a5842626c91ff75021dc10bdf6f8283a4bd19a39ed3e34392087fb600623.jpg)  
Fig. 4. Contact height detection module.

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/8fbdd8d5965c92bf2ab30473b32368a8aeef919639a15d1028fea5d4bed2ceb7.jpg)

the various measurements ((9), (10), (15), (16)) are included in Supplementary 5. Finally, the posterior state  $\bar{\mathbf{x}}_{k + 1}$  can be calculated through the Kalman update process

$$
\bar {\mathbf {x}} _ {k + 1} = \hat {\mathbf {x}} _ {k + 1} \boxplus \mathbf {K r} _ {k + 1} \tag {18}
$$

where  $\mathbf{K}$  is the Kalman filter gain [22] and  $\mathbf{R}$  is the covariance of Gaussian noise  $\mathbf{n}$ . And the posterior covariance  $\bar{\mathbf{P}}_{k + 1}$  can be updated as:

$$
\bar {\mathbf {P}} _ {k + 1} = \left(\mathbf {I} - \mathbf {K H} _ {k + 1}\right) \hat {\mathbf {P}} _ {k + 1} \tag {19}
$$

# IV. LIDAR ODOMETRY

# A. Adaptive Scan Slicing and Splicing

Spinning lidar usually takes a certain amount of time (100 ms) to accumulate a  $360^{\circ}$  FOV full scan, but this process time is too large and causes motion distortion. LLOL [23] and LoLa-SLAM [24] slice the scan according to a specific angle to decrease the latency. They use sliced scan as input instead of full scan, making it possible to get a high rate of lidar odometry.

We propose an adaptive scan slicing and splicing method to alleviate the affects of high-dynamic motion for legged robots. Based on the linear velocity  $v$  and angular velocity  $\omega$  obtained by the leg odometry and converted to the magnitude of vectors in the body frame, the scans are partitioned using an adaptive angle  $\theta_{s}$ :

$$
\sigma = \left(1 - \max  \left(\frac {\omega}{\omega_ {\operatorname* {m a x}}}, \frac {v}{v _ {\operatorname* {m a x}}}\right)\right) F o v \tag {20}
$$

$$
\theta_ {s} = \underset {m _ {i}} {\arg \min } | m _ {i} - \sigma |, m _ {i} \in \left[ \frac {1}{4}, \frac {1}{2}, 1 \right] F o v \tag {21}
$$

where  $v_{max}$  and  $\omega_{max}$  are the maximum linear and angular velocity of robot, and  $Fov$  is the lidar's  $360^{\circ}$  FOV. As shown in Fig. 5, a incoming full scan  $\mathcal{P}_f$  is partitioned into slices  $\{\mathcal{P}_{s1},\dots,\mathcal{P}_{sn}\}$ , which have the size of  $\theta_{s}$ . Since slice with smaller FOV will be inaccurate in point cloud matching, we input the historical slices into the buffer, and splice the time-continuous historical slices and current slice into a new full scan  $\mathcal{P}_f'$ . In the experiment, we consider that the high-frequency input will aggravate the optimization process, so the output frequency of the new full scan could only be increased to a maximum of  $40\mathrm{Hz}$  (25ms). Assume that the point  $q$  in the historical or current slices in a new full scan is sampled at time  $t_j$ . The point  $q$  can be

<sup>5</sup>https://github.com/ouguangjun/Leg-KILO/blob/main/supplementary.pdf

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/6094d59e94f4f4d53bf07701c1b2e40177dbb98039cf52dea5cd910331a8a8e4.jpg)  
Fig. 5. Overview of adaptive scan slicing and splicing. A incoming full scan is adaptively partitioned into slices based on the motion velocity estimated by leg odometry. Then, the current slice is spliced with historical slices to obtain a new full scan before ICP.

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/2ce74ae71f70156c69c59bdc4994270e45fc82cc66a50022df992604a2a8a3b8.jpg)  
Fig. 6. Robot-centric incremental map. The circular region search is used to determine which lidar keyframes need to be added or deleted from the map. In current search (yellow circle), new keyframes (blue dots) are detected and added to the map, while past keyframes (red dots) are not detected and removed from the map.

projected to the begin time  $t_i$  of the full scan as:  $q' = T_i^{-1}T_jq$ , where  $T_i$  and  $T_j$  are transformations from the body frame to the world frame, which are estimated by leg odometry.

# B. Odometry and Mapping

Similar to [17], planes and lines are extracted after receiving the de-skewed scan  $\mathcal{P}'$ . According to the initial guess provided by leg odometry, the extracted features will perform point-to-line  $\varPhi_{l}$  and point-to-plane  $\varPhi_p$  residual calculations with the corresponding features of the local map  $\mathcal{P}_{map}$ :

$$
\mathbf {r} = \sum_ {N _ {l}} ^ {k = 1} \Phi_ {l} \left(q _ {l, k}, \mathcal {P} _ {\text {m a p}}\right) + \sum_ {N _ {p}} ^ {k = 1} \Phi_ {p} \left(q _ {p, k}, \mathcal {P} _ {\text {m a p}}\right) \tag {22}
$$

where  $q_{l,k}, q_{p,k} \in \mathcal{P}'$ . After several iterations of the Gauss-Newton method, the residual gradually converges.

In this work, we introduce the lidar keyframe selection concept [3] to reduce the computation of map maintenance. A robot-centric incremental point cloud map is built for the point cloud registration and the contact height detection. As shown in Fig. 6, the circular region centered on the robot is a search area which is used to determine which keyframes need to be updated or deleted from the map. The keyframe will be added to the map or removed as the robot moves. Many works [3], [17] use static kd-tree for the storage of local map and point-wise operations. However, when updating the map, this static data structure needs to be rebuilt for the whole map, which is time-consuming. We employ incremental kd-tree (ikd-Tree) [25] to maintain the local map, which can supports incrementally adding and deleting

point cloud. In the meantime, it also maintains a downsampled map at a given resolution. Multi-thread parallel computation can improve the efficiency of tree re-balancing.

# V. FACTOR GRAPH OPTIMIZATION

We utilize factor graphs to tightly couple leg odometry, lidar odometry and loop closure fator. For loop closure, we follow [3] to apply an efficient distance-based loop closure detection. We choose the state corresponding to the lidar keyframes as the nodes of the factor graph. We define  $\mathcal{K}_{ij}$  and  $\mathcal{L}_{ij}$  are transformations of leg odometry and lidar odometry, respectively, between node  $i$  and node  $j$ . And  $\mathcal{O}_{mn}$  is defined as the measurement from ICP between two matched nodes  $m$  and  $n$ . When given the measurements, we can utilize the MAP method to estimate the set of states  $\mathcal{X}_k^*$ :

$$
\begin{array}{l} \mathcal {X} _ {k} ^ {*} = \underset {\mathcal {X} _ {k}} {\arg \min } \| \mathbf {r} _ {0} \| _ {\Sigma_ {0}} ^ {2} + \sum_ {(m, n) \in C _ {k}} \| \mathbf {r} _ {\mathcal {O} _ {m n}} \| _ {\Sigma_ {\mathcal {O} _ {m n}}} ^ {2} \\ + \sum_ {(i, j) \in \mathrm {K} _ {k}} \left(\left\| \mathbf {r} _ {\mathcal {K} _ {i j}} \right\| _ {\Sigma_ {\mathcal {K} _ {i j}}} ^ {2} + \left\| \mathbf {r} _ {\mathcal {L} _ {i j}} \right\| _ {\Sigma_ {\mathcal {L} _ {i j}}} ^ {2}\right) \tag {23} \\ \end{array}
$$

where  $\mathbf{r}$  is the residual between the measurement and the predicted value.  $\Sigma$  is the corresponding covariance.

# VI. EXPERIMENTS

We conducted extensive experiments with the Unitree Go1 legged robot both indoors and outdoors. The Unitree Go1 is a highly dynamic robot capable of achieving a maximum speed of  $2.0\mathrm{m / s}$  in our experiences. All experiments are conducted using the trot gait. The equipped with a on-board Velodyne VLP-16 Lidar, as well as a built-in IMU. Each foot of the robot is equipped with three joint encoders to measure joint angle and angular velocity, and a contact sensor for measuring ground contact force. Whether the foot contact is determined by the change of force. Our method is developed based on LIOSAM [3]. Detailed implementation and dataset can be found on our GitHub. The proposed method is tested on a desktop equipped with an Intel i5-12490F processor and 16GB of memory. Additionally, we also conduct runtime testing on the portable laptop with an Intel i5-7300HQ  $(2.50\mathrm{GHz} \times 4)$  CPU. At the beginning of each sequence, the robot remains stationary for a period of time to complete state initialization, such as gravity vector, as described in [18].

# A. Dataset

Due to the lack of an open source kinematic-inertial-lidar dataset for legged robots, we performed the robot using trot gait, and collected data in serval sequences, containing corridor, parking, slope, running, indoor, which varying in terms of structure, size, and motion speed. To obtain the ground truth, in sequence indoor, we registered the point cloud scan into the prior map by ICP to obtain the trajectory, where the prior map was collected by lidar statically. For other sequences, we employed an offline optimization approach where the previous estimation results were saved and were used as an initial guess for the

TABLE III COMPARISON OF RELATIVE POSE ERROR (RPE) FOR OUR PROPOSED METHOD AND OTHER METHODS  

<table><tr><td rowspan="2">Sequences (mean velocity [m/s])</td><td colspan="8">Mean Euler Angle Error [deg] / Translation Error [m]</td></tr><tr><td>KF [6]</td><td>ESKF4(Ours)</td><td>A-LOAM</td><td>FAST-LIO2</td><td>POINT-LIO</td><td>LIO-SAM</td><td>LIO-SAM5(with ESKF)</td><td>Ours (w/o LC)</td></tr><tr><td>corridor1(0.68)</td><td>0.76/0.75</td><td>0.95/0.63</td><td>0.74/0.18</td><td>1.06/0.28</td><td>1.00/0.23</td><td>×3</td><td>0.68/0.17</td><td>0.23/0.08</td></tr><tr><td>parking1(0.96)</td><td>0.54/1.08</td><td>1.11/0.95</td><td>0.87/0.23</td><td>1.03/0.26</td><td>1.09/0.24</td><td>×</td><td>0.78/0.20</td><td>0.17/0.08</td></tr><tr><td>indoor2(0.31)</td><td>0.71/0.10</td><td>0.54/0.08</td><td>0.45/0.04</td><td>0.84/0.09</td><td>0.72/0.06</td><td>0.47/0.04</td><td>0.47/0.04</td><td>0.15/0.03</td></tr><tr><td>running2(1.50)</td><td>0.67/0.17</td><td>1.26/0.15</td><td>1.08/0.08</td><td>2.07/0.19</td><td>1.73/0.09</td><td>1.10/0.09</td><td>1.03/0.10</td><td>0.07/0.05</td></tr></table>

The distance of RPE is set based on the sequence's distance  $(^{1}\mathrm{RPE}$  of  $10\mathrm{m},^2\mathrm{RPE}$  of  $1\mathrm{m})$  .  $^3\times$  denotes that the method failed.  
4 Our proposed kinematic-inertial leg odometry.  $^{5}$  LIO-SAM modified by leg odometry instead of the original IMU odometry.  
The bold values represent the best results.

current ICP algorithm, and the loop detection module was also highly utilized. Multiple iterations were performed until convergence was achieved, allowing us to obtain the high-accuracy results. We also verify methods for end-to-end error on sequence corridor, slope, and compare the variation of body height while operating on flat ground. More details about experiments are shown in a video https://youtu.be/6O74De5BLeQ.

# B. Comparison Results

For benchmark results, our proposed method is compared with another kinematic-inertial method (KF [6]), and other state-of-the-art lidar-based methods (A-LOAM [17], LIO-SAM [3], FAST-LIO2 [18], POINT-LIO [19]). KF is a popular state estimator which uses linear Kalman filter to fuse leg kinematics and IMU. For the implementation of KF, we directly use the orientation provided by the 9-axis IMU as the input of the filter. The prediction noise and measurement noise in KF and our proposed leg odometry (ESKF) are the same. Owing to high-frequency impact affecting the IMU accuracy, many lidar-inertial methods encounter significant drift or even breakdown. Consequently, for FAST-LIO2 and POINT-LIO, we increase the covariance of the accelerometer. In the case of LIO-SAM, we turn off the loop detection and additionally replace the original IMU odometry with our proposed leg odometry (LIO-SAM with ESKF). Unless otherwise specified, "Leg-KILO" and "Ours (w/o LC)" refer to versions without loop closure to allow a fair comparison with other algorithms that do not include loop closure. "Ours (full)" represents the version with all components included. We set the map downsampling rate to the same 0.3 for all methods. Note that the bold values in the tables represent the best results.

1) Accuracy Comparison: The comparison of mean Relative Pose Error (RPE) is shown in Table III. It can be seen that our leg odometry ESKF performs best in relative translation compared to another leg odometry KF. This is because ESKF uses the LiDAR map as an additional source of contact height measurements to reduce errors in body height. Due to the lack of observation of the yaw angle, it is difficult to obtain the high-accuracy orientation with leg odometry [4]. We find that the local accuracy of lidar-inertial methods that directly take unstable IMU measurements as input, such as FAST-LIO2 and LIO-SAM, experienced local fluctuations or breakdowns. Incorporating more kinematic constraints, our proposed kinematic-inertial-lidar method Leg-KILO outperforms other methods. For the yaw angle in rotation, the comparison results are shown in

TABLE IV COMPARISON OF RELATIVE YAW ANGLE ERROR [DEG]  

<table><tr><td>Method</td><td>corridor</td><td>parking</td><td>indoor</td><td>running</td></tr><tr><td>KF [6]</td><td>0.542</td><td>0.468</td><td>0.376</td><td>0.475</td></tr><tr><td>Ours (w/o LC)</td><td>0.277</td><td>0.215</td><td>0.183</td><td>0.095</td></tr></table>

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/ca7f397996dc42279129c5cc3fcc7a91c47659223282e52c26eea9c854cf8927.jpg)  
Fig. 7. Comparison of trajectory for various methods on parking.

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/76f88f693a3712c6076419606b9942ad9576d9a36b48f5516dfd1d6dbe0c1637.jpg)  
Fig. 8. Comparison of body height's variation for various methods while running on flat ground. The body height will drop slightly when running, and will not exceed the gray area in the figure.

Table IV, showing that the yaw angle can be better observed by incorporating an external sensor. The trajectory shown in Figs. 7 and 9 also validate the high local accuracy and global consistency of Leg-KILO. For comparison of end-to-end error, as shown in Table V, many methods obtain drifts less than  $0.01\%$  in sequence corridor. In sequence slope, the robot completed a loop and returned to its initial position in environments with long slopes (height variations exceeding 6 meters), as shown in Fig. 10. Compared to experiments conducted on flat terrain, the performance of our method is more apparent in scenarios with height variations. This is primarily attributed to the integration

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/e8740b6d9c3f9cba336ba0cf70baf9fd13b662300c0ba29d0d8ec157dcfaec3a.jpg)  
Fig. 9. Comparison of trajectory for various methods on slope.

TABLEV COMPARISON OF END-TO-END ERROR [M] FOR VARIOUS METHODS  

<table><tr><td></td><td>FAST LIO2</td><td>POINT LIO</td><td>LIO-SAM (with ESKF)</td><td>Ours CHD,LC)</td><td>Ours (w/o LC)</td><td>Ours (full)</td></tr><tr><td>corridor</td><td>0.20</td><td>0.03</td><td>0.10</td><td>0.12</td><td>0.04</td><td>0.03</td></tr><tr><td>slope</td><td>2.25</td><td>3.77</td><td>&gt;10</td><td>2.34</td><td>1.42</td><td>0.13</td></tr></table>

CHD: Contact Height Detection  
The bold values represent the best values

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/2719057cfd2b1be6d55f21ced9654fc1883ea37be22269b28fb89c5322b2a1d7.jpg)  
Fig. 10. Experimental results of legged robot on sequence slope. (a) The long slope with height variation (exceeding 6 meters). (b) (c) The trajectory with environment mapping results. (d) The foot contact points {LF, LH, RF, RH} measured from Contact Height Detection, which are shown in different colors.

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/868f9e21c3094b359c6e01b3977ceb91d84be217e3b8367162babe01c2c2dd05.jpg)  
Fig. 11. Comparison of incorporating contact height measurements into the filter at different times on leg odometry in sequence corridor. "full" indicates that every measurement is used, while "keyframe  $i$ s" indicates that measurements are only added at keyframes, where i represents the time interval between keyframes.

of leg kinematic constraints to reduce fluctuations in height caused by dynamic motion. The mapping results are shown in Fig. 12.

2) Body Height's Variation: In sequence running, the robot ran at an average speed of  $1.50\mathrm{m / s}$  on flat ground. The comparison of body height variations among different methods is shown

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/bdf8703197354b722ced93f6cb031d3e712d77625043b765802577630055c80c.jpg)  
Fig. 12. (a) and (b) show the global mapping results of the sequence corridor and parking. (c) and (d) are close-up views of the white boxes in (a) and (b), respectively.

TABLE VI ABLATION STUDY OF VARIOUS SLICE SIZES ON SEQUENCE RUNNING  

<table><tr><td></td><td>360°</td><td>180°</td><td>90°</td><td>Adaptive</td></tr><tr><td>ATE [m]</td><td>0.063</td><td>0.052</td><td>0.049</td><td>0.055</td></tr><tr><td>CPU usage [%]</td><td>33.62</td><td>50.30</td><td>60.81</td><td>45.73</td></tr></table>

The bold values represent the best values

in Fig. 8. Due to the robot's body experiencing slight downward motion during running, we observed height variations within a certain range (gray area in the figure). Several methods exhibit large drift during running and are unable to recover their normal heights during stationary periods. The body height measured by our method can vary within the normal range. Additionally, to investigate the extent to which contact height detection reduces errors in leg odometry, we examine the impact of incorporating contact height measurements into the filter at different stages in sequence corridor. The results are shown in Fig. 11. It can be observed that as the keyframe time interval increases (i.e., fewer observations are incorporated), the body height struggles to return to normal levels. This shows the positive effect of contact height detection in mitigating cumulative errors from dynamic movements.

3) Ablation Study: As shown in Table III, in LIO-SAM, the IMU odometry obtained by IMU integration exhibits significant drift and even breakdown during long-term operation. Replacing the original IMU odometry with leg odometry ESKF in LIO-SAM has resulted in a notable improvement. From the Table V, it can be observed that the removal of contact height detection (Section III-D3) module has resulted in significant drift, highlighting its critical role. The introduction of loop closure can further reduce global accumulated errors. We simultaneously compare different fixed slicing angles with adaptive slicing angles. From the Table VI, it can be observed that with smaller

![](https://cdn-mineru.openxlab.org.cn/result/2025-10-19/da957d02-bcf1-4f65-8809-e053b4bd85cf/3b0fd5f3cd73fa809e4a6f97e2e0d81362bf2e536a0fa2feaa35557efc80b691.jpg)  
Fig. 13. Runtime comparison on sequence parking.

slicing angles, there is a improvement in accuracy, but CPU usage significantly increases. This is because the ICP thread, which is the primary calculation, becomes more crowded as the number of inputs increases. As the robot's speed increases, the enhancement from slicing slices instead of full scans becomes more pronounced. Adaptive slicing based on the robot's velocity can balance accuracy and computational resources, which is crucial for robots operating with limited computational power.

4) Efficiency: The runtime comparison on a portable laptop is shown in Fig. 13. We can find that our method can run at a average rate below 50ms. The processing time of LIO-SAM increases as the map size grows and exceeds 100ms. This is because, in LIO-SAM, the map reconstruction during each update consumes a considerable amount of time, whereas our method allows for incremental updates.

# VII. CONSLUSION

We propose a Kinematic-Inertial-Lidar Odometry method, Leg-KILO. The integration of leg kinematics constraints enhances the stability of lidar-based odometry. An adaptive scan slicing and splicing method is introduced to alleviate the effect of dynamic motion. Additionally, a robot-centric incremental local mapping approach reduces map maintenance time costs. Factor graph optimization is employed to integrate leg odometry, lidar odometry, and loop closure factors to improve accuracy. Our method is extensively tested in various experiences involving high-dynamic motion using trot gait, demonstrating greater suitability for dynamic quadruped motion compared to other state-of-the-art lidar-based methods. Maintaining robot-centric local maps affects the accuracy of contact height detection. In the future, we will pay more attention to improving the consistency of maps to improve the accuracy of contact height detection and point cloud registration.

# REFERENCES

[1] T. Kim, S. Kim, and D. Lee, "Tunable impact and vibration absorbing neck for robust visual-inertial state estimation for dynamic legged robots," IEEE Robot. Automat. Lett., vol. 8, no. 3, pp. 1431-1438, Mar. 2023.  
[2] B. R. P. Singh and R. Featherstone, "Mechanical shock propagation reduction in robot legs," IEEE Robot. Automat. Lett., vol. 5, no. 2, pp. 1183-1190, Apr. 2020.  
[3] T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and D. Rus, "Lio-SAM: Tightly-coupled lidar inertial odometry via smoothing and mapping," in 2020 IEEE/RSJ Int. Conf. Intell. Robots Syst., 2020, pp. 5135-5142.

[4] M. Bloesch et al., "State estimation for legged robots: Consistent fusion of leg kinematics and IMU," Robot.: Sci. Syst. VIII, 2013, pp. 17-24, doi: 10.7551/mitpress/9816.001.0001.  
[5] M. Bloesch, C. Gehring, P. Fankhauser, M. Hutter, M. A. Hoepflinger, and R. Siegwart, "State estimation for legged robots on unstable and slippery terrain," in 2013 IEEE/RSJ Int. Conf. Intell. Robots Syst., 2013, pp. 6058-6064.  
[6] G. Bledt, M. J. Powell, B. Katz, J. D. Carlo, P. M. Wensing, and S. Kim, "MIT cheetah 3: Design and control of a robust, dynamic quadruped robot," in 2018 IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 2245-2252.  
[7] R. Hartley, M. Ghaffari, R. M. Eustice, and J. W. Grizzle, "Contact-aided invariant extended Kalman filtering for robot state estimation," Int. J. Robot. Res., vol. 39, no. 4, pp. 402-430, 2020.  
[8] J. Ma, M. Bajracharya, S. Susca, L. Matthies, and M. Malchano, "Real-time pose estimation of a dynamic quadruped in GPS-denied environments for 24-hour operation," Int. J. Robot. Res., vol. 35, no. 6, pp. 631-653, 2016.  
[9] D. Wisth, M. Camurri, and M. Fallon, "Robust legged robot state estimation using factor graph optimization," IEEE Robot. Automat. Lett., vol. 4, no. 4, pp. 4507-4514, Oct. 2019.  
[10] S. Yang, Z. Zhang, Z. Fu, and Z. Manchester, "Cerberus: Low-drift visual-inertial-leg odometry for agile locomotion," in 2023 IEEE Int. Conf. Robot. Automat., 2023, pp. 4193-4199.  
[11] D. Wisth, M. Camurri, and M. Fallon, “Preintegrated velocity bias estimation to overcome contact nonlinearities in legged robot odometry,” in 2020 IEEE Int. Conf. Robot. Automat., 2020, pp. 392-398.  
[12] Y. Kim, B. Yu, E. M. Lee, J.-H. Kim, H. W. Park, and H. Myung, "STEP: State estimator for legged robots using a preintegrated foot velocity factor," IEEE Robot. Automat. Lett., vol. 7, no. 2, pp. 4456-4463, Apr. 2022.  
[13] V. Dhédin et al., “Visual-inertial and leg odometry fusion for dynamic locomotion,” in 2023 IEEE Int. Conf. Robot. Automat., 2023, pp. 9966–9972.  
[14] D. Wisth, M. Camurri, and M. Fallon, "VILENS: Visual, inertial, lidar, and leg odometry for all-terrain legged robots," IEEE Trans. Robot., vol. 39, no. 1, pp. 309–326, Feb. 2023.  
[15] J.-H. Kim et al., "Legged robot state estimation with dynamic contact event information," IEEE Robot. Automat. Lett., vol. 6, no. 4, pp. 6733-6740, Oct. 2021.  
[16] R. Hartley, M. G. Jadidi, L. Gan, J.-K. Huang, J. W. Grizzle, and R. M. Eustice, "Hybrid contact preintegration for visual-inertial-contact state estimation using factor graphs," in 2018 IEEE/RSJ Int. Conf. Intell. Robots Syst., 2018, pp. 3783-3790.  
[17] J. Zhang and S. Singh, "Low-drift and real-time lidar odometry and mapping," Auton. Robots, vol. 41, pp. 401-416, 2017.  
[18] W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, “FAST-LIO2: Fast direct LiDAR-inertial odometry,” IEEE Trans. Robot., vol. 38, no. 4, pp. 2053–2073, Aug. 2022.  
[19] D. He, W. Xu, N. Chen, F. Kong, C. Yuan, and F. Zhang, “Point-LIO: Robust high-bandwidth light detection and ranging inertial odometry,” Adv. Intell. Syst., vol. 5, 2023, Art. no. 2200459.  
[20] M. Tranzatto et al., “CERBERUS in the DARPA Subterranean Challenge,” Sci. Robot., vol. 7, no. 66, 2022, Art. no. eabp9742, doi: 10.1126/scirobotics.abp9742  
[21] B. Morrell et al., "NeBula: TEAM coSTAR's robotic autonomy solution that won phase II of DARPA subterranean challenge," Field Robot., vol. 2, pp. 1432-1506, 2022.  
[22] P. S. Maybeck, Stochastic Models, Estimation, and Control, vol. 3. Cambridge, MA, USA: Academic Press, 1982.  
[23] C. Qu, S. S. Shivakumar, W. Liu, and C. J. Taylor, "LLOL: Low-latency odometry for spinning LiDARs," in 2022 Int. Conf. Robot. Automat., 2022, pp. 4149-4155.  
[24] M. Karimi, M. Oelsch, O. Stengel, E. Babaians, and E. Steinbach, "Lola-SLAM: Low-latency LiDAR SLAM using continuous scan slicing," IEEE Robot. Automat. Lett., vol. 6, no. 2, pp. 2248-2255, Apr. 2021.  
[25] Y. Cai, W. Xu, and F. Zhang, "ikd-Tree: An incremental KD tree for robotic applications," 2021, arXiv:2102.10808.