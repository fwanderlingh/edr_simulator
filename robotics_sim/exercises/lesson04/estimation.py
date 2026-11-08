"""Compare encoder odometry and inertial integration on the same sensor stream.

Choose Lesson 4 > Compare encoder and IMU, select the student estimator,
then save your implementation and click Reload exercise.
Complete the two Student.update methods after the guided exercise in the notes.
"""

from ._runtime import InertialState, OdometryState


# Select the component to implement, then save and click Reload exercise.
# The other estimate is explicitly the supplied reference for comparison.
ESTIMATION_STAGE = "both"  # "encoder", "imu", or "both"


class StudentOdometry(OdometryState):
    def update(self, left_delta, right_delta, dt):
        """Update x,y,yaw,speed,yaw_rate using wheel increments [rad] and dt [s].

        Geometry is self.radius [m], self.width [m]; all states start at zero.
        Use constant-arc or midpoint integration, as in Lesson 3.
        """
        # TODO: wheel increments -> ds, da -> pose; never use ground truth here.
        raise NotImplementedError("Complete StudentOdometry.update in lesson04/estimation.py")


class StudentInertial(InertialState):
    def update(self, gyro, ax_body, ay_body, dt, *, truth_yaw=None):
        """Update yaw,x,y,vx,vy; body acceleration is already in m/s² without gravity.

        truth_yaw=None: propagate yaw from gyro. Otherwise use supplied yaw only
        for the explicitly orientation-assisted comparison. All states start at 0.
        """
        # TODO: yaw -> acceleration in world -> position and velocity integration.
        raise NotImplementedError("Complete StudentInertial.update in lesson04/estimation.py")
