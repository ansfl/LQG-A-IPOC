&nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp;  <img src="https://github.com/ansfl/MEMS-IMU-Denoising/blob/main/figrues/Logo.png?raw=true" width="500" />


### Introduction
Linear quadratic Gaussian (LQG) control is a well-established method for optimal control through state estimation, particularly in stabilizing an inverted pendulum on a cart. In standard laboratory setups, sensor redundancy enables direct measurement of configuration variables using displacement sensors and rotary encoders. However, in outdoor environments, dynamically stable mobile platforms—such as Segways, hoverboards, and bipedal robots—often have limited sensor availability, restricting state estimation primarily to attitude stabilization. Since the tilt angle cannot be directly measured, it is typically estimated through sensor fusion, increasing reliance on inertial sensors and necessitating a lightweight, self-contained perception module. 

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
 <img src="https://github.com/Daniboy370/Inertial_LQG/blob/main/data/Fig_Diagram_4.png?raw=true" width="700" class='center'/>

Despite its effectiveness, prior research has not incorporated accelerometer data into the LQG framework for stabilizing pendulum-like systems, as jerk states are not explicitly modeled in the Newton-Euler formalism. This paper addresses this limitation by leveraging local differential flatness to incorporate linear and angular jerk states into the system model:

&nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
 <img src="https://github.com/Daniboy370/Inertial_LQG/blob/main/data/Fig_Augmented_0.png?raw=true" width="1000" class='center'/>

The resulting higher-order system dynamics enhance state estimation, leading to a more robust LQG controller capable of predicting accelerations for dynamically stable mobile platforms : 

&nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
 <img src="https://github.com/Daniboy370/Inertial_LQG/blob/main/data/Fig_Augmented_1.png?raw=true" width="1000" class='center'/>

This refinement improves overall control performance, as demonstrated in the following simulation:

&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
 <img src="https://github.com/Daniboy370/Inertial_LQG/blob/main/data/Vid_Pend_1.gif?raw=true" width="950" class='center'/>


### Simulation Results

To evaluate our hypothesized higher-order system model, we conduct a comprehensive assessment. First, the left figure verifies state solutions under ideal conditions, with a continuous prediction-to-update ratio ($\rho=1$). Then, the right figure examines the same configuration variables and their derivatives under lower ratios ($\rho<1$), where external updates are scarcer and more discontinuous:

 &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
 <img src="https://github.com/Daniboy370/Inertial_LQG/blob/main/data/Fig_Comp_1.png?raw=true" width="1150" class='center'/>

We analyze the impact of update ratios on observer-controller (KF-LQR) sensitivity. At $\rho = 0.5$ (top row), small, bounded errors appear, with the rightmost column showing the corresponding control action. This performance stems from the predictive model compensating for uncertainty. At $\rho = 0.1$ (middle row), greater update discontinuity thickens and adds noise to error patterns, increasing control fluctuations as it struggles to stabilize configuration variables. The response remains stable but less smooth, requiring more control effort. At $\rho = 0.01$ (bottom row), the controller switches abruptly between saturation boundaries, indicating instability. State estimates diverge, causing drift and insufficient control feedback, ultimately leading to unrecoverable perturbations.

 &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
 <img src="https://github.com/Daniboy370/Inertial_LQG/blob/main/data/Fig_Ratio.png?raw=true" width="700" class='center'/>

To enhance our temporal analysis, both variables are normalized by their initial conditions $\dot{x}_0, \dot{\theta}_0$, and then against the normalized time axis. This transformation enhances interpretability, ensuring that all trajectories originate from the light blue sphere at (1,1,1). If stabilization is achieved, they converge to the red sphere at the origin (0,0,0); otherwise, they diverge.  

 &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
 <img src="https://github.com/Daniboy370/Inertial_LQG/blob/main/data/Fig_Comp_2.png?raw=true" width="1000" class='center'/>

Next, we conduct an empirical sensitivity analysis, presenting four response surfaces in a clockwise sequence: (i) final cart position $x(T)$, (ii) final pendulum angle $\theta(T)$, (iii) total control effort $U_{tot}$, and (iv) actuator saturation percentage over time $u_{sat}$.  

 &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
 <img src="https://github.com/Daniboy370/Inertial_LQG/blob/main/data/Fig_Comp_3.png?raw=true" width="1000" class='center'/>

As observed, both models exhibit similar patterns with subtle differences. However, two key aspects reveal significant contrasts. First, the A-IPoC model consistently maintains a larger stability region ${S}_{A-IPoC} > {S}_{IPoC}$. Second, the level sets expand differently, with the IPoC model exhibiting a higher divergence level. These differences suggest that A-IPoC offers greater marginal stability by moderating divergence for the same initial perturbation. Quantitatively, A-IPoC outperforms IPoC, with stability areas 27\%-39\% larger and crash rates 10\%-15\% lower, demonstrating superior disturbance immunity and control resilience.
 
## Code

For convenience, both inference and training notebooks are provided, GPU-required.

* **Full Mode** - Full solution pipeline from training to inference. 

* **Test Mode** - Direct inference and comparison between competing models, by uploading pretrained model weights, trained over *single Nvidia T4 GPU*. 

### Directory tree
<pre>
[root directory]
├── code
|   ├── IPoC_Init
|   ├── IPoC_Linearize
|   ├── IPoC_Solve
|   ├── IPoC_main
    ...
|   ├── Convergence_main
    └── Stability_main
├── data
...
└── requirements.txt
<!--  Readme.md -->
</pre>

File | Purpose | File | Purpose 
--- | --- | --- | ---
**IPoC_main** | IPoC Launcher file | **A_IPoC_main** | A-IPoC Launcher file
**IPoC_Init** | IPoC model initialization (conventional) | **A-IPoC_Init** | A-IPoC model initialization (ours)
**IPoC_Linearize** | Regular linearization process | **A_IPoC_Linearize** | Augmented linearization process
**IPoC_Solve** | Numerical solution (using RK-45) | **A_IPoC_Solve** | Numerical solution (using RK-45)

**Convergence_main**: A comparative analysis of spatial trajectories between the two competing models.

**Stability_main**: A comparative sensitivity analysis of both models at various prediction-update ratios.

## Citation

The authors would appreciate users giving stars to this repository and citing our article as follows:
```
@article{engelsman2025InertialBasedLQG,
  title={Inertial-Based LQG Control: A New Look at Inverted Pendulum Stabilization},
  author={Engelsman, Daniel and Klein, Itzik},
  journal={arXiv preprint arXiv:2312.12121},
  year={2025}
}
```

[<img src=https://upload.wikimedia.org/wikipedia/commons/thumb/a/a8/ArXiv_web.svg/250px-ArXiv_web.svg.png width=70/>](https://arxiv.org/abs/2312.12121)
