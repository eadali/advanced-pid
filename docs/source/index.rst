Data-Driven Control
*************************************
Control Theory for humans.  
The PID controller design based entirely on experimental data collected from the plant.

.. image:: ../../imgs/star.png 
Please Star me on GitHub for further development.

.. contents:: Table of Contents
   :depth: 2
   
   
PID Controller Example
-------------------------
PIDController class can be used directly if the controller gains are already calculated. PIDController class is based on Thread Class. So it can be used as Tread. This features provides fixed PID loop frequency.

.. code-block:: python

    from ddcontrol.model import TransferFunction
    from ddcontrol.control import PIDController
    import numpy as np
    import matplotlib.pyplot as plt
    import time
    
    #Creates PID controller and test model
    pid = PIDController(kp=30, ki=70.0, kd=0.0, kn=0.0)
    ref = 1.0
    tf = TransferFunction([1.0], [1.0,10.0,20.0], udelay=0.1)
    
    #Control loop
    history = []
    u = 0.0
    pid.start()
    start = time.time()
    for _ in range(1000):
        t = time.time() - start
        y = tf.step(t, u)
        u = pid.update(ref-y)
        history.append([t,y])
        time.sleep(0.001)
    
    #Stops PID controller
    pid.stop()
    pid.join()
    
    #Plots result
    np_hist = np.array(history)
    fig, ax = plt.subplots()
    ax.plot(np_hist[:,0], np_hist[:,1])
    ax.grid()
    plt.show()

Controlled output:  

.. image:: ../../imgs/output1.png

PID optimization for known Transfer Function
-------------------------
If the transfer function is already known, controller gains can be calculated by pidopt method.

.. code-block:: python

    from ddcontrol.model import TransferFunction
    from ddcontrol.control import pidopt
    import numpy as np
    import matplotlib.pyplot as plt
    import time
    
    #Creates transfer function
    tf = TransferFunction([1.0], [1.0,10.0,20.0], udelay=0.1)
    
    #Optimize PID controller
    pid, _ = pidopt(tf)
    ref = 1.0
    
    #Control loop
    history = []
    u = 0.0
    pid.start()
    start = time.time()
    for _ in range(1000):
        t = time.time() - start
        y = tf.step(t, u)
        u = pid.update(ref-y)
        history.append([t,y])
        time.sleep(0.001)
    
    #Stops PID controller
    pid.stop()
    pid.join()
    
    #Plots result
    np_hist = np.array(history)
    fig, ax = plt.subplots()
    ax.plot(np_hist[:,0], np_hist[:,1])
    ax.grid() 
    plt.show()

Controlled output:  

.. image:: ../../imgs/output2.png 

Transfer Function Estimation for unknown SISO system
-------------------------
If the transfer function is unknown for system, the transfer function can be estimated by tfest method.

.. code-block:: python

    from ddcontrol.model import TransferFunction, tfest
    import numpy as np
    import matplotlib.pyplot as plt
    
    #Creates a transfer function and input output data
    tf = TransferFunction([1.0], [1.0,10.0,20.0], 1.0)
    t, y, u = np.linspace(0,10,101), np.zeros(101), np.ones(101)
    for index in range(t.size):
        y[index] = tf.step(t[index], u[index])
    
    #Predicts transfer function
    tf_est, _ = tfest(t, y, u, np=2, nz=0, delay=True)
    y_est = np.zeros(101)
    for index in range(t.size):
        y_est[index] = tf_est.step(t[index], u[index])
    
    #Plots result
    fig, ax = plt.subplots()
    ax.plot(t, y, '.-', label='Real')
    ax.plot(t, y_est, '.-', label='Estimated')
    ax.legend()
    ax.grid()
    plt.show()

Step response of real system and estimated system:  

.. image:: ../../imgs/output3.png 

Installation
-------------------------
To install using pip

    pip install ddcontrol

ddcontrol\.control module
-------------------------

.. automodule:: ddcontrol.control
    :members:
    :undoc-members:
    :show-inheritance:

ddcontrol\.model module
-----------------------

.. automodule:: ddcontrol.model
    :members:
    :undoc-members:
    :show-inheritance:    

ddcontrol\.integrate module
---------------------------

.. automodule:: ddcontrol.integrate
    :members:
    :undoc-members:
    :show-inheritance:
