# element_aerodynamics-control
## Responsible:
|                          Hui Tong                     | 
:-----------------------------------------------------:|
<img src="https://avatars.githubusercontent.com/u/74595921?v=4" width="180">|

## Background

The existing achievements of control algorithm with the reference as [IEEE-RAL 2018 momentum-based controller](https://arxiv.org/abs/1702.06075) applied on flying iRonCub robot are based on the assumption that the aerodynamic effects are negligible. This project aims to analyse the aerodynamic effects on flying iRonCub robot which is managed to be a further step of [element_ironcub-control](https://github.com/dic-iit/element_ironcub-control#design-a-controller-that-considers-aerodynamic-effects-and-jets-dynamics). Combined with the results from CFD analysis which contributes to the simulated aerodynamic force that can be applied on the  robot model, theoretically available controllers and following simulation are expected to be achieved. Further more ,experimentals would be organized based on the achievments of proper designed controllers and simulation which could bring the `Flying Humanoid Robot` closer to the reality. 

## Objectives
### - To build robot models with aerodynamic effects
### - To implement aerodynamic forces in current simulator
### - To redesign the [controller](https://arxiv.org/abs/1702.06075) by considering aerodynamic forces


## Outcomes
The possible outcomes of this element:

### - Simplified aerodynamic models to evaluate aerodynamic effects on iRonCub

### - A simulator of flying iRonCub contains aerodynamics forces

### - A controller that stabilizes the flying iRonCub with aerodynamic effects

### - Publication for aerodynamic-flight-control

## Milestones

The following milestones have been identified for this project:

### - Identify the model of aerodynamic forces on iRonCub with CFD analysis,[achieved](https://github.com/ami-iit/element_aerodynamics-control/issues/42);
### - Implementation of aerodynamic models on Matlab-Based simulator, [achieved](https://github.com/ami-iit/element_aerodynamics-control/issues/56);
### - Proceed robustness tests of current controller with aerodynamics effects, [achieved](https://github.com/ami-iit/element_aerodynamics-control/issues/34);
### - Improve the current controller to take care of aerodynamics effects on iRonCub, [achieved](https://github.com/dic-iit/element_aerodynamics-control/issues/20);
### - Prepare a publication of the thesis work for ICRA 2022, [in progress](https://github.com/ami-iit/element_aerodynamics-control/issues/65).

For furhter introduction of the repo concpets, please see the documentation in [wiki](https://github.com/ami-iit/element_aerodynamics-control/wiki).
