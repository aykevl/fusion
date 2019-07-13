# IMU sensor fusion algorithms for Go

[![GoDoc](https://godoc.org/github.com/aykevl/fusion?status.svg)](https://godoc.org/github.com/aykevl/fusion)

This package implements sensor fusion algorithms for Go. At the moment, the
following algorithms have been implemented:

  * [Madgwick](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/): an
    advanced filter close to Kalman in performance but much easier to use and
    much lower on resources. It can also run at a low frequency: 10Hz updates
    are still usable.  
    At the moment, only the IMU version has been implemented (gyroscope +
    accelerometer), the AHRS version (with added magnetometer) is not yet
    implemented.

## License

This package as a whole is licensed under the BSD 2-clause license, see the
LICENSE file for details. Some code is placed in the public domain, where
indicated in the file itself.
