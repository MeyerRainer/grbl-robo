GRBL 0.9j modified for robot manipulator controlling purposes.

Works only on Arduino Mega (AtMega2560).

Install and configure like any other GRBL.

Additional features:
  Controls 8 axes. Currently, number of axis is not configurable. If not used, they can just be ignored in G-code and will stay zero.
  Axis letters are [X, Y, Z, A, B, C, U, V].

  Servo control for gripper control:
  The PWM signal on pin D2 can be controlled via G-code as follows:
  G6P0 -> 1ms pulse width
  G6P0.5 -> 1.5ms pulse width
  G6P1 -> 2ms pulse width

  32-bit unsigned integer timestamp on realtime status report
  Running milliseconds are reported in the status report, allowing the control software do speed estimation.

  For connecting hardware, take a look at "cpu_map_atmega2560.h".

CNC-specific features removed:
-G2,G3 arc motion
-CoreXY kinematics


