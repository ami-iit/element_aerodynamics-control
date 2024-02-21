# element_aerodynamics-control
## Responsible:

[Antonello Paolino](https://github.com/antonellopaolino)     |
:-------------------------:|
<img src="https://github.com/antonellopaolino.png" width="180"> |  

## Background

The existing achievements of control algorithm with the reference as [IEEE-RAL 2018 momentum-based controller](https://arxiv.org/abs/1702.06075) applied on flying iRonCub robot are based on the assumption that the aerodynamic effects are negligible. This project aims to analyse the aerodynamic effects on flying iRonCub robot which is managed to be a further step of [element_ironcub-control](https://github.com/dic-iit/element_ironcub-control#design-a-controller-that-considers-aerodynamic-effects-and-jets-dynamics). Combined with the results from CFD analysis (detailed in [element_cfd-simulation](https://github.com/ami-iit/element_cfd-simulation) repo) which contributes to the simulated aerodynamic force that can be applied on the  robot model, theoretically available controllers and following simulation are expected to be achieved. Further more, experiments would be organized based on the achievements of proper designed controllers and simulation which could bring the `Flying Humanoid Robot` closer to the reality. 

## Objectives

* **Identify aerodynamic force models for the robot**
* **Implement aerodynamic models in current simulator**
* **Redesign the [controller](https://arxiv.org/abs/1702.06075) including aerodynamic modeling**


## Outcomes
Possible outcomes of this element:

* **Aerodynamic models to evaluate aerodynamic effects on iRonCub**

* **A simulator of flying iRonCub considering aerodynamics forces**

* **A controller that stabilizes the flying iRonCub with aerodynamic effects**

* **Publications for aerodynamic-flight-control**


## Milestones

* **Identify the model of aerodynamic forces on iRonCub with CFD analysis, [achieved](https://github.com/ami-iit/element_aerodynamics-control/issues/42)**
* **Implementation of aerodynamic models on Matlab-Based simulator, [achieved](https://github.com/ami-iit/element_aerodynamics-control/issues/56)**
* **Proceed robustness tests of current controller with aerodynamics effects, [achieved](https://github.com/ami-iit/element_aerodynamics-control/issues/34)**
* **Improve the current controller to take care of aerodynamics effects on iRonCub, [achieved](https://github.com/dic-iit/element_aerodynamics-control/issues/20)**
* **Prepare a publication of the thesis work for ICRA 2022, [achieved](https://github.com/ami-iit/element_aerodynamics-control/issues/65)**
* **Test and refine the whole-body controller accounting for aerodynamics, [ongoing](https://github.com/ami-iit/element_aerodynamics-control/issues/92)**


## Wiki

For further information please refer to the documentation in [wiki](https://github.com/ami-iit/element_aerodynamics-control/wiki).
