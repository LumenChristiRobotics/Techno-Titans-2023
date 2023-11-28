package org.firstinspires.ftc.teamcode.common.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumDriveCustom extends RobotDrive {
    private double rightSideMultiplier;
    private double revsPerInch = 10;

    DcMotor[] motors;


    public MecanumDriveCustom(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this(true, frontLeft, frontRight, backLeft, backRight);
    }

    /**
     * Sets up the constructor for the mecanum drive.
     *
     * @param autoInvert Whether or not to automatically invert the right motors
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    public MecanumDriveCustom(boolean autoInvert, DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        setRightSideInverted(autoInvert);
    }

    public boolean isRightSideInverted() {
        return rightSideMultiplier == -1.0;
    }

    public void setRightSideInverted(boolean isInverted) {
        rightSideMultiplier = isInverted ? -1.0 : 1.0;
    }

    /**
     * Sets the range of the input, see RobotDrive for more info.
     *
     * @param min The minimum value of the range.
     * @param max The maximum value of the range.
     */
    public void setRange(double min, double max) {
        super.setRange(min, max);
    }
    public void setMaxSpeed(double value) {
        super.setMaxSpeed(value);
    }

    @Override
    public void stop() {
        for (DcMotor x : motors) {
            x.setPower(0);
        }
    }


    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0);
    }


    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param squareInputs Square joystick inputs for finer control
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean squareInputs) {
        strafeSpeed = squareInputs ? clipRange(squareInput(strafeSpeed)) : clipRange(strafeSpeed);
        forwardSpeed = squareInputs ? clipRange(squareInput(forwardSpeed)) : clipRange(forwardSpeed);
        turnSpeed = squareInputs ? clipRange(squareInput(turnSpeed)) : clipRange(turnSpeed);

        driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro
     */
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle) {
        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turnSpeed = clipRange(turnSpeed);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[MotorType.kFrontLeft.value] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[MotorType.kFrontRight.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[MotorType.kBackLeft.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[MotorType.kBackRight.value] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[MotorType.kFrontLeft.value] += turnSpeed;
        wheelSpeeds[MotorType.kFrontRight.value] -= turnSpeed;
        wheelSpeeds[MotorType.kBackLeft.value] += turnSpeed;
        wheelSpeeds[MotorType.kBackRight.value] -= turnSpeed;

        normalize(wheelSpeeds);

        driveWithMotorPowers(
                wheelSpeeds[MotorType.kFrontLeft.value],
                wheelSpeeds[MotorType.kFrontRight.value],
                wheelSpeeds[MotorType.kBackLeft.value],
                wheelSpeeds[MotorType.kBackRight.value]
        );
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param xSpeed       the horizontal speed of the robot, derived from input
     * @param ySpeed       the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro
     * @param squareInputs Square the value of the input to allow for finer control
     */
    public void driveFieldCentric(double xSpeed, double ySpeed, double turnSpeed, double gyroAngle, boolean squareInputs) {
        xSpeed = squareInputs ? clipRange(squareInput(xSpeed)) : clipRange(xSpeed);
        ySpeed = squareInputs ? clipRange(squareInput(ySpeed)) : clipRange(ySpeed);
        turnSpeed = squareInputs ? clipRange(squareInput(turnSpeed)) : clipRange(turnSpeed);

        driveFieldCentric(xSpeed, ySpeed, turnSpeed, gyroAngle);
    }

    /**
     * Drives the motors directly with the specified motor powers.
     *
     * @param frontLeftSpeed    the speed of the front left motor
     * @param frontRightSpeed   the speed of the front right motor
     * @param backLeftSpeed     the speed of the back left motor
     * @param backRightSpeed    the speed of the back right motor
     */
    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        motors[MotorType.kFrontLeft.value]
                .setPower(frontLeftSpeed * maxOutput);
        motors[MotorType.kFrontRight.value]
                .setPower(frontRightSpeed * rightSideMultiplier * maxOutput);
        motors[MotorType.kBackLeft.value]
                .setPower(backLeftSpeed * maxOutput);
        motors[MotorType.kBackRight.value]
                .setPower(backRightSpeed * rightSideMultiplier * maxOutput);
    }


    public void driveForwardInInches(int numberOfInches, double speed) {
        double totalRevs = revsPerInch * numberOfInches;

        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontLeft.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kFrontRight.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kBackLeft.value].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[MotorType.kBackRight.value].setDirection(DcMotorSimple.Direction.FORWARD);

        motors[MotorType.kFrontLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kFrontRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
        motors[MotorType.kBackLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kBackRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
    }

    public void driveBackwardInInches(int numberOfInches, double speed) {
        double totalRevs = revsPerInch * numberOfInches * -1;

        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontLeft.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kFrontRight.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kBackLeft.value].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[MotorType.kBackRight.value].setDirection(DcMotorSimple.Direction.FORWARD);

        motors[MotorType.kFrontLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kFrontRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
        motors[MotorType.kBackLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kBackRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
    }

    public void turnLeftNinetyDegrees(double speed) {
        double totalRevs = revsPerInch * 5 * -1;

        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontLeft.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kFrontRight.value].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[MotorType.kBackLeft.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kBackRight.value].setDirection(DcMotorSimple.Direction.FORWARD);

        motors[MotorType.kFrontLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kFrontRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
        motors[MotorType.kBackLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kBackRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
    }

    public void turnRightNinetyDegrees(double speed) {
        double totalRevs = revsPerInch * 5 * -1;

        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontLeft.value].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[MotorType.kFrontRight.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kBackLeft.value].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[MotorType.kBackRight.value].setDirection(DcMotorSimple.Direction.REVERSE);

        motors[MotorType.kFrontLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kFrontRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
        motors[MotorType.kBackLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kBackRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
    }

    public void strafe(double numberOfInches, double speed) {
        double totalRevs = revsPerInch * numberOfInches;

        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[MotorType.kFrontLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackLeft.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kBackRight.value].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[MotorType.kFrontLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackLeft.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kBackRight.value].setTargetPosition((int)totalRevs);
        motors[MotorType.kFrontLeft.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kFrontRight.value].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[MotorType.kBackLeft.value].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[MotorType.kBackRight.value].setDirection(DcMotorSimple.Direction.REVERSE);

        motors[MotorType.kFrontLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kFrontRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
        motors[MotorType.kBackLeft.value]
                .setPower(speed * maxOutput);
        motors[MotorType.kBackRight.value]
                .setPower(speed * rightSideMultiplier * maxOutput);
    }

}