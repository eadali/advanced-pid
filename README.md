[![Run Tests and Publish PyPI](https://github.com/eadali/advanced-pid/actions/workflows/main-branch.yml/badge.svg)](https://github.com/eadali/advanced-pid/actions/workflows/main-branch.yml)
# advanced-pid    
An advanced PID controller in Python. The derivative term can also be used in 
practice thanks to built-in first-order filter. Detailed information can be
found [here](https://en.wikipedia.org/wiki/PID_controller#Derivative_term).

Usage is very simple:

```python
from advanced_pid import PID

# Create PID controller 
pid = PID(Kp=2.0, Ki=0.1, Kd=1.0, Tf=0.05)

# Control loop
while True:
    # Get current measurement from system
    timestamp, measurement = system.get_measurement()
    
    # Calculate control signal by using PID controller
    reference = 1.0
    control = pid(timestamp, reference - measurement)
    
    # Feed control signal to system
    system.set_input(control)
```

Complete API documentation can be found 
[here](https://advanced-pid.readthedocs.io/en/latest/).

## Usage
Biggest advantage of advanced-pid, the derivative term has a built-in first-order
filter.    
advanced-pid package includes a toy mass-spring-damper system model for testing:

```python
from advanced_pid import PID
from advanced_pid.models import MassSpringDamper

# Create a mass-spring-damper system model
system = MassSpringDamper(mass=1.0, spring_const=1.0, damping_const=0.2)
system.set_initial_value(initial_position=1.0, initial_velocity=0.0)

# Create PID controller 
pid = PID(Kp=1.0, Ki=0.0, Kd=2.0, Tf=0.5)

# Control loop
time, meas, cont = [], [], []
for i in range(800):
    # Get current measurement from system
    timestamp, measurement = system.get_measurement()
    
    # Calculate control signal by using PID controller
    control = pid(timestamp, -measurement)
    
    # Feed control signal to system
    system.set_input(control)
    
    # Record for plotting
    time.append(timestamp)
    meas.append(measurement)
    cont.append(control)

# Plot result
import matplotlib.pyplot as plt
plt.subplot(2,1,1)
plt.title('Mass-Spring-Damper system')
plt.xlabel('Time[s]')
plt.ylabel('Measured Position [m]')
plt.plot(time, meas, 'r')
plt.grid()
plt.subplot(2,1,2)
plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.plot(time, cont, 'b')
plt.grid()
plt.show()
```
    
Output:  
![alt text](https://raw.githubusercontent.com/eadali/advanced-pid/main/docs/imgs/figure.png)

## Installation
To install, run:
```
pip3 install advanced-pid
```
## Tests
To run tests, run:
```
python3 test.py
```

## License
Licensed under the 
[MIT License](https://github.com/eadali/advanced-pid/blob/main/LICENSE.md).
