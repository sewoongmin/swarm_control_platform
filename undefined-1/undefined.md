# 충동회피 알고리즘

## 최적화 기법 ( Optimization methods )

Optimization methods attempt to find an input that minimizes a performance index to avoid obstacles \[13], \[14]. Most of these methods calculate the performance index for a finite time horizon, which can be easily combined with model predictive control as in \[15]–\[19]. However, receding horizon control has some inherent disadvantages coming from the absence of state information over the finite time horizon. Smith et al. developed a general framework to pose a collision avoidance problem of remotely piloted aircrafts as an optimal control problem using a stochastic estimator \[20], \[21], where they used particle filter to minimize the effects of uncertainty caused by pop-up circumstances and to enable real-time implementation. Sujit and Beard developed a path planning algorithm for multiple UAVs with limited sensor and communication ranges \[22]. They used particle swarm optimization to avoid detected static and pop-up obstacles.

Optimal Reciprocal Collision Avoidance (ORCA), Model Predictive Control (MPC)



### 단점

실시간성이 떨어짐&#x20;

However, these optimization-based approaches are computationally intensive and requires heuristic choice of a termination criterion to guarantee a convergence time and, therefore, is yet difficult to apply for real-time operations.

## 인공전위장 ( potential-field approach )

The force-field approach, or potential-field approach, assumes virtual fields around obstacles. Virtual attractive or repulsive forces created by these fields are used to generate collision avoidance maneuvers \[29]–\[32]. However, local minima may exist in the force field and cannot be easily addressed. Koren and Borenstein presented inherent problems of potential field methods \[33] as oscillation in a high-speed real-time system and trap situations due to local minima. Paul et al. \[34] and Chen et al. \[35] found similar local minima problems when using an extended artificial potential field for formation flight ofUAVs. The local minima were related with a length of time step and force gradient.

### 단점

local minima

## 감지 후 회피 (The sense-and-avoid approach)

The sense-and-avoid approach,which is essentially similar to a pilot’s behavior in a manned aircraft, prevents a collision by changing the direction of travel of the aircraft away from the obstacle. The simplicity of the sense-andavoid approach results in low computational requirements and short response times. The sense-and-avoid approach is more advantageous for UAVs than for manned aircraft, because UAVs usually have low-quality sensors for obstacle detection. Hence, research on sense-and-avoid method for UAV has been initiated since the late 1990 s. Prats et al. reviewed regulations, recommended practices, and standards in sense-and-avoid methods for unmanned aircraft \[36]. Chakravarthy and Ghose proposed a collision-cone approach for collision detection and avoidance between irregularly shaped moving objects. The collision-cone approach can be effectively used to examine a condition of the collision between a robot and a moving obstacle \[37]. Goss et al. considered a collision avoidance problem between two aircraft in a three-dimensional (3-D) environment using a combination of a geometric approach and a collision cone approach, and they proposed a guidance law based on the collision-cone approach \[38]. In \[39], a minimum effort guidance law was developed to guide a UAV to a waypoint while avoiding multiple obstacles using the collision-cone approach. A 2-D passive vision system and an extended Kalman filter were used to estimate the positions of the obstacles.

## 기하학적인  방식 (a collision avoidance algorithm based on geometry)

![](<../.gitbook/assets/image (45).png>)

## Gridding approach

## Genetic algorithm
