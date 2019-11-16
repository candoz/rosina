# Robotic Swarm in ARGoS

## About the project

This is the final project for the [Intelligent Robotic Systems course](https://www.unibo.it/en/teaching/course-unit-catalogue/course-unit/2018/384267) by professor [Andrea Roli](https://www.unibo.it/sitoweb/andrea.roli).  

Our goal is to reproduce part of the work done by Shervin Nouyan, Alexandre Campo and Marco Dorigo regarding **robotic chain formations** with the aim of deepening the subject, reworking the controller logic and finally simulate it in an ARGoS environment. Link to the article: [Path formation in a robot swarm](https://link.springer.com/article/10.1007/s11721-007-0009-6).

## How to run the simulation

* Install the [ARGoS simulator](http://www.argos-sim.info)
* `git clone https://github.com/candoz/rosina.git`
* `argos3 -c rosina.argos`

Note: you may find more information on programming robots in Lua with ARGoS at http://www.argos-sim.info/plow2015/

## Results

We provide a [detailed report](report.pdf) containing design choices, considerations and tests.

A demo of the final application is provided below. The main goal is to form a chain of bots starting from the nest and arriving to one of the preys. The white dot represents the nest, while the black dots are meant to be preys.  
It's configured with a swarm size of 30 robots and a noise level of 0.3 on all the sensors and actuators.

![Chain formation demo](res/chain-formation.gif "Chain formation demo")

## License
RoSinA project is licensed under the [GPL 3.0 License](https://www.gnu.org/licenses/gpl-3.0.html).
