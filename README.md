# Communication
Communication is made via NRF24L01 26 bytes at a time, them being:

- [VX] (00-03) Desired velocity in the x axis (4 bytes / float)
- [VY] (04-07) Desired velocity in the y axis (4 bytes / float)
- [VT] (08-11) Desired angular velocity (4 bits / float)
- [KS] (12-12) Kick signal (1 byte / bool)
- [Kp] (13-16) Kp constant for angular velocity PID (4 bytes / float)
- [Kp] (17-20) Ki constant for angular velocity PID (4 bytes / float)
- [Kp] (21-24) Kd constant for angular velocity PID (4 bytes / float)

# Transmitter

- Iteration duration: 64 ms
- Frequency: 15.625 hz