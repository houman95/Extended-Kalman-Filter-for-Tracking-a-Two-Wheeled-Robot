Extended-Kalman-Filter-for-Tracking-a-Two-Wheeled-Robot


An Extended Kalman Filter was designed for tracking the position and orientation of a two-wheeled robot that is moving on a plane. The Estimator.m implements the estimation algorithm. At time tk = kT, the estimator has access to the time tk, the control inputs uL[k] and uR[k], and
possibly the measurements of its position and orientation
that are corrupted by sensor noise zx[k], zy[k], or zr[k]. The robot can command its left and right wheel angular velocities, uL(t) and uR(t) (in rad/s),
at discrete time instants t0 = 0, t1 = T, t2 = 2T, . . . , where T is the sampling time in s. The
commands are kept constant in between sampling instants and are assumed to be followed
instantaneously. Non-idealities in the actuation mechanism (for example, wheel slip or
imperfect command tracking) are considered: they are modeled as multiplicative noise on the
actuation commands.
