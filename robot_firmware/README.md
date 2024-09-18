# Robot-firmware modules
- [ ] Radio Communication
- [ ] Control
- [ ] Gyroscope
- [x] Motor activation
- [x] Serial communication (Debugging)

# Communication
Communication is made via NRF24L01 64 bits at a time, them being:

![Comm bits visualization](../resources/comm_bits_definition.png "Bits visualization")
- [VX] 63-48 Desired velocity in the x axis (16 bits)
- [VY] 47-32 Desired velocity in the y axis (16 bits)
- [DO] 31-17 Desired orientation of the robot (15 bits)
- [NU] 16-16 Not used (1 bit)
- [CO] 15-1  Current orientation of the robot from vision (15 bits)
- [KS]  0-0  Kick signal (1 bit)
