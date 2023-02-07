# Gyroscope Localization

In this project, we tried to estimate the location of a moving object using a gyroscope. Using a 9-axis gyroscope, we get the linear and rotational acceleration and use that to estimate the location.

## Instruments

We used Raspberry Pi 3b as mainboard and MPU9250 for gyroscope. We had connected these module using a breadboard.

## How to run

Run project using command

```bash
py final_version.py
```

You can change hyperparameter above code.

```Python
ALGO_NAME = 'Madgwick'
```

## Algorithms

We have used three algorithms to convert relative accelerations to absolute accelerations.

- [Mahony](https://ahrs.readthedocs.io/en/latest/filters/mahony.html): This estimator proposed by Robert Mahony et al. is formulated as a deterministic kinematic observer on the Special Orthogonal group SO(3) driven by an instantaneous attitude and angular velocity measurements.

- [Madjwick](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html): This is an orientation filter applicable to IMUs consisting of tri-axial gyroscopes and accelerometers, and MARG arrays, which also include tri-axial magnetometers, proposed by Sebastian Madgwick.

- [Extended Callman Filter](https://ahrs.readthedocs.io/en/latest/filters/ekf.html): The Extended Kalman Filter is one of the most used algorithms in the world, and this module will use it to compute the attitude as a quaternion with the observations of tri-axial gyroscopes, accelerometers and magnetometers.

## Denoising

We have used several techniques to denoise the input data. We mention some of them below.

- Calibrization: We first have a few rounds of rambling where we calibrate the input from the gyro assuming the location is fixed.

- Softening: We use two techniques to smooth the inputs to reduce transient noise. At the start of the execution, you can choose one of these two modes.

  + Exponensial Average: We use the exponential average in such a way that we consider an average value and with the entry of each new value, we update this value as follows. $x_{mean}' = (1-\alpha)x_{mean} + \alpha x$
  
  + Sliding Window: We used an sliding window to consider average of last 10 record isntead of last 10.

## Video

We have recorded a video in which we have explained the project and its function. You can access the video in the link below.

<https://drive.google.com/file/d/1VZW6sKtyKiKR5tcF6zujy4zgZAD5IC6E/view?usp=share_link>

## Developers

Aryan Ahadinia

Sepehr Pourghanad

Mostafa Ojaghi
