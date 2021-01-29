Extended-Kalman-Filter-for-Tracking-a-Two-Wheeled-Robot


An Extended Kalman Filter was designed for tracking the position and orientation of a two-wheeled robot that is moving on a plane. The Estimator.m implements the estimation algorithm. At time tk = kT, the estimator has access to the time tk, the control inputs uL[k] and uR[k], and
possibly the measurements zx[k], zy[k], or zr[k].  Non-idealities in the actuation mechanism (for example, wheel slip or
imperfect command tracking) are considered: they are modeled as multiplicative noise on the
actuation commands.
