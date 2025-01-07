This project explores the design and implementation of advanced controllers for a crane system
with two suspended loads. The system consists of a crane moving along a frictionless track,
with two masses, m1 and m2, suspended via cables of lengths l1 and l2 respectively as shown
in Fig. 1.1. The goal is to control the system effectively while analyzing its controllability
and observability properties.
![image](https://github.com/user-attachments/assets/1f09c26c-c89d-42cf-9c18-13f6bdbca6f7)

The process begins with deriving the equations of motion for the system using the
Lagrangian method. These equations are then used to construct a nonlinear state-space
representation. To simplify the analysis and controller design, the system is linearized around
a given equilibrium point, and the linearized state-space representation is derived.

Next, the project examines the conditions under which the linearized system is controllable
based on parameters such as the masses and cable lengths. With these insights, an LQR
(Linear Quadratic Regulator) controller is designed to stabilize the system. The designed
controller is tested on both the linearized and original nonlinear models through simulations
using Matlab and Simulink. The stability of the closed-loop system is then analyzed using
Lyapunov’s indirect method.

Following the LQR design, a Luenberger observer is developed for the system, considering
different observable output configurations. The final task is to design an output feedback
controller using the LQG (Linear Quadratic Gaussian) method.

This approach combines mathematical modeling, control design, and simulation to tackle
the challenges of controlling a multi-mass crane system, demonstrating how modern control
methods can be applied effectively.
