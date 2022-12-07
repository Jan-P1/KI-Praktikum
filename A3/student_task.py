import airsim
from manager import Manager
import numpy as np
from numpy import dot
from numpy.linalg import inv


class KalmanFilter:
    """Locates a car in a 2 dimensional world.

        Attributes:
            delta_seconds: The sample time in seconds.
            x: The estimated state vector.
            C: The output matrix.
            A: The system matrix.
            R: The measurement noise covariance matrix.
            Q: The process noise matrix. (not mentioned in lecture, in your initial solution you can ignore this matrix)
            P: The error covariance matrix.
            K: The kalman gain matrix.
    """

    def __init__(self, x: np.ndarray, delta_seconds: float, gps_stddev_x: float, gps_stddev_y: float, imu_stddev: float):
        """ Initialize KalmanFilter

        Args:
            x (np.ndarray): The initial state estimate
            delta_seconds: The time between every update
            gps_stddev_x: standard deviation of the x-coordinate of the GPS sensor
            gps_stddev_y: standard deviation of the y-coordinate of the GPS sensor
            imu_stddev: standard deviation of the acceleration sensor, that is inside the IMU
        """

        self.delta_seconds = delta_seconds

        # ****************************************************************
        # 1. Define the state vector of the car
        # ****************************************************************
        # You can define the initial state vector in the main function below.
        # TODO: x = np.array([state1, state2, ...]).T
        self.x = x

        # ****************************************************************
        # 2. Define the output matrix C
        # ****************************************************************
        # TODO: Determine which states you measure with the sensors.
        self.C = np.array([[1.0, 0.0, ...],
                           [0.0, 1.0, ...],
                           [..., ..., ...]])

        # ****************************************************************
        # 3. Define the system matrix
        # ****************************************************************
        # TODO: Define, with the motion model of your car, the system matrix.
        self.A = np.array([[..., ...],
                           [..., ...]])

        # ****************************************************************
        # 4. Define the measurement noise covariance matrix.
        # ****************************************************************
        # The R matrix indicates the inaccuracy of our measurement vector y.
        # TODO: Add the variance for the measurement noise of each sensor.
        self.R = np.diag([..., ...])

        # ****************************************************************
        # 5. The process noise matrix
        # ****************************************************************
        # The Q matrix indicates the inaccuracy of our motion model. (this matrix was not mentioned in lecture)
        # for your first implementation of your Kalman filter, you do not have to use this matrix. Later on, you should
        # use it. You can find information about this matrix in the script about the Kalman filter in ilias or anywhere in
        # the Internet.
        # The Q matrix has the same dimension as the P and A matrix.
        # TODO: Test with different values. What influence does the Q-Matrix have on the estimation of the Kalman Filter?
        # TODO: Q = np.diag([variance for state1, variance for state2, ...])
        self.Q = np.diag([..., ...])

        # ****************************************************************
        # 6. The initial error covariance matrix P
        # ****************************************************************
        # TODO: Determine the error of the initial state estimate.
        # TODO: dx = np.array([standard deviation from the first state, ...])
        dx = np.array([gps_stddev_x, gps_stddev_y, ...])

        self.P = np.dot(dx.T, dx)

        # *****************************************************************
        # 7. The control input matrix (OPTIONAL, as not mentioned in lecture)
        # *****************************************************************
        # TODO: Determine how much the control_input changes each state vector component.
        # You can implement the Kalman filter at the beginning without the control input and the B matrix.
        self.B = np.array([[..., ...],
                           [..., ...]])

        # Kalman matrix
        self.K = None

    def update(self, y: np.ndarray, control_input: np.ndarray) -> np.ndarray:
        """ Updates the state of the Kalman Filter with new sensor data

        Args:
            y: The measurement vector with the measurements from our sensors [gps_x, gps_y, ...].T
            control_input: The current throttle or brake input from our car in the x and y direction.
                           The control input is a 2 dimensional vector [x, y].T.
                           The values can be between 1 and -1. control_input[0] = 1 means that the car drives with
                           maximum acceleration in the positive direction on the x-axis.
                           Can also be replaced with the last acceleration measurement.
        Returns:
            The estimated state of the car.
        """

        # Prediction step
        # TODO: Implement the prediction step. Update x with the motion model and calculate P.

        # Correction step
        # TODO: Implement the Correction step. Correct the prediction with the measurements.

        return self.x


if __name__ == '__main__':

    manager = Manager()

    activ = True

    while activ:

        # ****************************************************************
        # The localization done only with the GPS sensor
        # Should be replaced with the Kalman filter
        # ****************************************************************
        # TODO: Implement your Kalman Filter here and update the manager with the Kalman Filter estimates of postition and velocity

        # Get GPS data
        gps_position = manager.get_gps_position()
        gps_velocity = manager.get_gps_velocity()

        # Update car state
        activ = manager.update(gps_position.x_val, gps_position.y_val, gps_velocity.x_val, gps_velocity.y_val)
