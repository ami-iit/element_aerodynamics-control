## Aerodynamic Modeling and Control Software

Here you can find all the software developed to model, evaluate and validate the effects produced by the aerodynamic forces acting on a flying humanoid robot (in the specific `iRonCub-Mk1` and `iRonCub-Mk3` robot models).


#### [Aerodynamic flight controllers](./aerodynamic-flight-controllers/)

These controllers are built based on the [momentum-based-flight-sim-no-gazebo](https://github.com/ami-iit/ironcub-mk1-software/tree/main/flight-controllers-stable/momentum-based-flight-sim-no-gazebo) controller from https://github.com/ami-iit/ironcub-mk1-software, while the aerodynamic simulator is a modified version of the [matlab-whole-body-simulator](https://github.com/ami-iit/matlab-whole-body-simulator). The **aerodynamic-flight-controllers** aim at evaluating the aerodynamic effects on iRonCub and study possible control techniques to handle aerodynamic effects.

* In [centroidal-aerodynamics-flight](./aerodynamic_flight_controllers/centroidal-aerodynamics-flight), centroidal aerodynamic forces are introduced into the simulator and the controller. The controller is tested with three types of flight trajectories performed by `iRonCub-Mk1` robot. 

* In [whole-body-aerodynamics-flight](/aerodynamic_flight_controllers/whole-body-aerodynamics-flight), whole-body aerodynamic forces are introduced to the simulator and to the controller. The controller is tested with two types of trajectories and different combinations of simulator/controller aerodynamic models. 

* In [whole-body-aerodynamics-balancing](/aerodynamic_flight_controllers/whole-body-aerodynamics-balancing), whole-body aerodynamic forces are introduced to the real robot torque controller. The controller has been validated on the robot balancing while subject to a fictitious wind. 

Refer to the [wiki page](https://github.com/ami-iit/element_aerodynamics-control/wiki#aerodynamic-flight-controller) for info on how to run the aerodynamic-flight-controllers.


#### [Aerodynamic modeling](./aerodynamic-modeling/)

The aerodynamic models are divided into centroidal and whole-body aerodynamic models. They allow to have a model of the aerodynamic forces acting on iRonCub robot when subject to a non-zero relative wind velocity.

* In [centroidal-models](./aerodynamic-modeling/centroidal-models/), centroidal aerodynamic models have been identified to model the aerodynamic effects on a simplified (cylindroid) version of flying humanoid robot via linear regression from CFD data. 

* In [whole-body-models](./aerodynamic-modeling/whole-body-models/), whole-body aerodynamic models have been identified to model the distributed aerodynamic forces acting at each link of the flying humanoid robot:

    - [cfd-models](./aerodynamic-modeling/whole-body-models/cfd-models/) which are built using Linear/Lasso regression and Deep Neural Network on data coming from CFD simulations on `iRonCub-Mk1` model.
    
    - [symmetrical-link-models](./aerodynamic-modeling/whole-body-models/symmetrical-link-models/) which are built for a simplified axisymmetric robot model and are based on cylinder and sphere aerodynamic forces estimation from literature.


#### [Wind tunnel](./wind-tunnel/)

Here you can find the code used to control the `iRonCub-Mk1` and `iRonCub-Mk3` robots in the wind tunnel campaigns at GVPM facility of Politecnico di Milano.

* In [joint-configuration-generation](./wind-tunnel/joint-configuration-generation/), there are matlab scripts to generate automatic random joint configurations close to the reference positions, accountign also for locked joints,

* In [position-control](./wind-tunnel/position-control/), there is the position controller used to move the robot in the desired generated joint positions to test it and acqurie the aerodynamic force data.


### Related publications:

[1] [Momentum Control of an Underactuated Flying Humanoid Robot](https://doi.org/10.1109/LRA.2017.2734245)

[2] [Position and Attitude Control of an Underactuated Flying Humanoid Robot](https://doi.org/10.1109/HUMANOIDS.2018.8624985)

[3] [Centroidal Aerodynamic Modeling and Control of Flying Multibody Robots](https://doi.org/10.1109/ICRA46639.2022.9812147)
