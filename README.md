# rc-controls

Actuation and controls stack for RC cars. Generates PWM and servo inputs to feed to the rc-cars.

## Installation

```
cd my_ws/src
git clone git@github.com:cornellev/rc-controls.git
```

## Instructions

To run just this package: `roslaunch launch controls.launch`. 

This will run one node that accepts ackermann drive inputs and generates inputs for the car.

