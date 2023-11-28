package org.firstinspires.ftc.teamcode.opmodes.teleop;

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.subsystems.drive.MecanumDriveCustom;
import org.firstinspires.ftc.teamcode.common.subsystems.intake.gaberClaw;
import org.firstinspires.ftc.teamcode.common.subsystems.lift.Shoulder;
import org.firstinspires.ftc.teamcode.common.subsystems.lift.Wrist;

@TeleOp(name="titanDrive", group = "Drive Modes")
public class titanDrive extends LinearOpMode {

    //Falloff and steepness values for the power curve. For more information check https://www.desmos.com/calculator/hgdyiukaoh
    private final double falloff = 2;
    private final double steepness = 2;

    private DcMotorEx fL, fR, bL, bR;
    private Gamepad driverController;
    private Gamepad lifterController;

    private MecanumDriveCustom mecanumDriveCustom;

    private gaberClaw theGaber;
    DcMotorEx shoulder1;
    DcMotorEx elbow;
    IMU imu;

    private Wrist wristMover;
    private Shoulder shoulderMover;

    @Override
    public void runOpMode() {

        // Initialize Variables and Drive Motors
        fL = hardwareMap.get(DcMotorEx.class,"leftFront");
        fR = hardwareMap.get(DcMotorEx.class,"rightFront");
        bL = hardwareMap.get(DcMotorEx.class,"leftRear");
        bR = hardwareMap.get(DcMotorEx.class,"rightRear");

        resetMotors(fL, fR, bL, bR);

        mecanumDriveCustom = new MecanumDriveCustom(fL, fR, bL, bR);

        fR.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse the front Motors

        driverController = gamepad1;
        lifterController = gamepad2;

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT, //Orthogonal #9 in the docs
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        //this here set it to radians before, however in mecanumDrive.driveFieldCentric(), it converts it to radians, so we will use degrees
        imu.initialize(parameters);
        imu.resetYaw();

        //Initialize the Gaber
        theGaber = new gaberClaw(hardwareMap.get(Servo.class, "largeClaw"), hardwareMap.get(Servo.class, "smallClaw"));
        theGaber.closeSmallClaw();
        theGaber.closeLargeClaw();
        //Initialize the Arm
        shoulder1 = hardwareMap.get(DcMotorEx.class, "shoulder1");

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");

        shoulderMover = new Shoulder(shoulder1);
        wristMover = new Wrist(elbow);

        mecanumDriveCustom.setMaxSpeed(20);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            //Drive Code
            //double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //mecanumDrive.driveFieldCentric(
            //        -adjustPower(driverController.left_stick_x),
            //        adjustPower(driverController.left_stick_y),
            //       -adjustPower(driverController.right_stick_x),
            //        heading
            //);
            //End Drive Code

            double y = -driverController.left_stick_y; // Remember, Y stick value is reversed
            double x = driverController.left_stick_x;
            double rx = driverController.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (driverController.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            fL.setPower(frontLeftPower);
            bL.setPower(backLeftPower);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);



            //Lifter Code
            if (gamepad2.y) {
                //moveArm(750);
                //moveWrist(328);
                shoulderMover.raiseArm();
                wristMover.raiseWrist();
            }

            if (gamepad2.a) {
                theGaber.openSmallClaw();
                theGaber.openLargeClaw();
                shoulderMover.lowerArm();
                wristMover.lowerWrist();
            }

            if (gamepad2.x) {
                theGaber.closeSmallClaw();
                theGaber.closeLargeClaw();
                sleep(750);
                shoulderMover.setDrivePosition();
                wristMover.lowerWrist();
            }

            if (gamepad2.b) {
                theGaber.openLargeClaw();
                sleep(500);
                theGaber.openSmallClaw();
                sleep(100);
            }



            //End Lifter Code
            //Drone Code
            if (gamepad2.dpad_up) {



            }
            //End Drone Code

            //The Gaber Code
            if (lifterController.left_bumper) {
                theGaber.openSmallClaw();
            }
            if (lifterController.right_bumper) {
                theGaber.closeSmallClaw();
            }
            if (lifterController.left_trigger>0) {
                theGaber.openLargeClaw();
            }
            if (lifterController.right_trigger>0) {
                theGaber.closeLargeClaw();
            }
            //End Gaber Code
        }
    }


    public void moveWrist(Integer position) {
        wristMover.moveToPosition(position);
    }

    public void moveArm(Integer position) {
        shoulderMover.moveToPosition(position);
    }

    //The adjusted power curve. For more information check https://www.desmos.com/calculator/hgdyiukaoh
    public double adjustPower(double input) {
        return MathUtils.clamp(input * Math.pow(falloff, Math.pow(input, 2 * steepness) - 1), -0.95, 0.95);
    }

    public void resetMotors(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.resetDeviceConfigurationForOpMode();
        }
    }

}
