package fusion

// This implementation of the Madgwick filter has been written by Ayke van
// Laethem and has been placed in the public domain. You can do with it whatever
// you want, but credit is appreciated.

import (
	"math"
	"time"

	"github.com/go-gl/mathgl/mgl32"
)

// The Madgwick filter integrates gyroscope and accelerometer data. According to
// the author, it is close to the Kalman filter in accuracy but is far cheaper
// to compute.
type Madgwick struct {
	// Estimated orientation.
	mgl32.Quat

	// beta (or β) is precomputed from the gyroscope measurement error provided
	// in NewIMU().
	beta float32
}

// NewMadgwick returns a new Madgwick filter.
//
// The gyroMeasurementError is the "estimated mean zero gyroscope measurement
// error", in other words, the average error of the gyroscope (in radians per
// second). The bigger this value, the more is looked at the accelerometer
// instead of the gyroscope. This means that a bigger value results in faster
// corrections of gyro drift at the expense of bigger distortion from the
// accelerometer.
func NewMadgwick(gyroMeasurementError float32) Madgwick {
	// Calculate β, see equation 50.
	beta := float32(math.Sqrt(3.0/4.0)) * gyroMeasurementError
	return Madgwick{
		Quat: mgl32.QuatIdent(),
		beta: beta,
	}
}

// Update the internal state of the filter with the new incoming values. It
// expects gyroscope (rotation) measurements in radians per second. Acceleration
// values are provided in g. The time delta is the time since the previous
// measurement (important to correctly integrate gyroscope measurements). For
// example, if you have a sample rate of 10Hz the time delta would be 100ms.
//
// The resulting orientation is stored as a quaternion in the Quat member of
// this struct.
func (m *Madgwick) Update(gyro, accel mgl32.Vec3, timeDelta time.Duration) {
	// The following algorithm has been based on the following paper:
	//   http://x-io.co.uk/res/doc/madgwick_internal_report.pdf
	// There is a possibly more up-to-date version available here, but it only
	// seems to leave some parts out:
	//   https://scholar.google.nl/scholar?cluster=12858071719902504630
	// For more information:
	//   http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
	// References are based on the former PDF ("An efficient orientation filter
	// for inertial and inertial/magnetic sensor arrays" by Sebastian O.H.
	// Madgwick).
	//
	// The basic idea is that we assume an initial attitude (orientation) and
	// update that attitude each interval with the gyroscope readings. The
	// gyroscope basically just tells how fast and in which direction we are
	// rotating. Integrating all those rotations over time should in theory give
	// us the actual attitude, weren't it that the initial attitude is likely to
	// be wrong and that the gyroscope readings contain errors that accumulate
	// over time.
	// To avoid all these errors, we use the accelerometer readings that are
	// often incorrect in the short term but more reliable when averaging the
	// readings over a longer time: the accelerometer doesn't just measure
	// gravity but also acceleration of the object itself. Also, accelerometers
	// are generally quite noisy. The complementary filter is essentially this:
	// for each update it adds the gyroscope readings to the estimated attitude
	// and adjusts it a little bit with the gyroscope readings. The Madgwick
	// filter is a bit more complex and does some more massaging of all these
	// numbers to correctly calculate the gyroscope error and subtract it each
	// iteration from the estimated orientation, leading to a better estimate
	// over time.
	// Take a look at figure 2 in the paper to better understand how the
	// algorithm is put together.
	//
	// Another note:
	// While the paper (and the original code) orders a 4-element vector as {W,
	// X, Y, Z}, this implementation uses the {X, Y, Z, W} ordering as that is
	// what is used inside the mathgl/mgl32 package.

	// Create a gyroscope quaternion from a raw gyroscope measurement, as in
	// equation 10. This simplifies working with it, as most other values are in
	// quaternion form.
	gyroQuat := mgl32.Quat{V: gyro}

	// Integrate the gyroscope readings into the current orientation estimate to
	// get a new orientation estimate (equation 12). This new orientation
	// estimate includes gyro drift errors which will be removed in a later
	// step.
	newOrientationWithError := m.Quat.Scale(0.5).Mul(gyroQuat)

	// Now that we have a first guess as to what the new orientation will be,
	// remove gyro drift using the accelerometer.

	// First of all, normalize the accelerometer measurements. We only care
	// about the direction of the acceleration, not the magnitude.
	accel = accel.Normalize()

	// Objective function, see equation 25.
	f := mgl32.Vec3{
		2*(m.X()*m.Z()-m.W*m.Y()) - accel.X(),
		2*(m.W*m.X()+m.Y()*m.Z()) - accel.Y(),
		2*(0.5-m.X()*m.X()-m.Y()*m.Y()) - accel.Z(),
	}

	// Jacobian, see equation 26.
	j := mgl32.Mat4x3FromCols(
		mgl32.Vec4{
			2 * m.Z(),
			-2 * m.W,
			2 * m.X(),
			-2 * m.Y(),
		},
		mgl32.Vec4{
			2 * m.W,
			2 * m.Z(),
			2 * m.Y(),
			2 * m.X(),
		},
		mgl32.Vec4{
			-4 * m.X(),
			-4 * m.Y(),
			0,
			0,
		},
	)

	// Compute the objective function gradient, as seen in equation 34.
	// Note that equation 34 has two forms: one when only using the
	// accelerometer (the upper equation) and one when using both the
	// accelerometer and the magnetometer (the lower equation). In this case we
	// only have accelerometer measurements so we use the upper equation, which
	// is essentially just a matrix multiplication.
	// Reinterpret the resulting gradient (which is a mgl32.Vec4) as a
	// quaternion for further processing.
	gradient := vec4ToQuat(j.Mul3x1(f))

	// The direction of the gyroscope error, see equation 44.
	directionOfError := gradient.Normalize()

	// Estimated rate of change of orientation with the error removed, see
	// equation 43.
	rateOfChange := newOrientationWithError.Sub(directionOfError.Scale(m.beta))

	// Integrate the rate of change, as shown in equation 42.
	// Also normalize the quaternion because simply adding some value to the
	// quaternion causes it to be non-normalized.
	timeDeltaSec := float32(timeDelta/time.Nanosecond) / 1000000000
	m.Quat = m.Quat.Add(rateOfChange.Scale(timeDeltaSec)).Normalize()
}

// vec4ToQuat reinterprets a 4-element vector as a quaternion. It is a no-op
// change.
func vec4ToQuat(v mgl32.Vec4) mgl32.Quat {
	return mgl32.Quat{v.W(), mgl32.Vec3{v.X(), v.Y(), v.Z()}}
}
