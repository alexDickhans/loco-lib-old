# LocoLib

## What is this?

This library is designed to help you will all things vex and localization. Now what is localization?
Localization is all the ways that your robot knows where it is on the field. Most often this is done with 
odometry deadwheels(Wheels that touch the ground and measure the movement of the robot without slip), however
this can often make robots much bulier if not harder to package. This library is designed to work with or
without odometry wheels and still be able to get, and keep a reliable position on the field(3 in error) at a
high(100 Hz) refresh rate. 

## How does it work

It isn't done yet but ideally we will be able to fuse odometry readings from multiple different sources such
as odometry wheels internal motor encoders, accelerometers, gyros, distance sensors, and GPS into one coherent,
reliable measurement.

## Laundry list

- [X] Create abstract deadwheel odometry class
- [ ] Implement 2/3 wheel odometry
- [ ] Implement drift drive odometry
- [X] Create particle filter
- [ ] Integrate distance sensor
- [ ] Integrate GPS sensor
- [ ] Kalman filter for orientation
- [ ] Keep good documentation
