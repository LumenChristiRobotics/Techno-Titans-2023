package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.common.states.AutonomousMode;
import org.firstinspires.ftc.teamcode.common.subsystems.drive.MecanumDriveCustom;
import org.firstinspires.ftc.teamcode.common.subsystems.intake.gaberClaw;
import org.firstinspires.ftc.teamcode.common.subsystems.lift.Shoulder;
import org.firstinspires.ftc.teamcode.common.subsystems.lift.Wrist;
import org.firstinspires.ftc.teamcode.common.vision.pipelines.PropDetection;
import org.firstinspires.ftc.teamcode.common.vision.util.PropPosition;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name="RedRight")
public class RedRight extends LinearOpMode {

    public List<LynxModule> modules;
    public boolean purplePixelScored = false;
    public OpenCvWebcam webcam;
    public PropDetection propDetectionPipeline;
    //public AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public gaberClaw theGaber;

    static final double FEET_PER_METER = 3.28084;

    private DcMotorEx fL, fR, bL, bR;

    public double fx = 822.317;
    public double fy = 822.317;
    public double cx = 319.495;
    public double cy = 242.502;

    public double tagsize = 0.166;
    public SampleMecanumDrive drive;
    public MecanumDriveCustom titanDrive;

    //CHANGE BASED ON POSITION
    public int LEFT = 2;
    public int MIDDLE = 6;
    public int RIGHT = 16;

    public AprilTagDetection tagOfInterest = null;

    public AutonomousMode.Side side;

    DcMotorEx shoulder1;
    DcMotorEx elbow;
    Integer sleepTime = 300;
    private Wrist wristMover;
    private Shoulder shoulderMover;

    //private driveToAprilTagID aprTag;

    @Override
    public void runOpMode() {
        // Initialize Variables and Drive Motors
        fL = hardwareMap.get(DcMotorEx.class,"leftFront");
        fR = hardwareMap.get(DcMotorEx.class,"rightFront");
        bL = hardwareMap.get(DcMotorEx.class,"leftRear");
        bR = hardwareMap.get(DcMotorEx.class,"rightRear");

        //Initialize the Arm
        shoulder1 = hardwareMap.get(DcMotorEx.class, "shoulder1");

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");

        shoulderMover = new Shoulder(shoulder1);
        wristMover = new Wrist(elbow);



        resetMotors(fL, fR, bL, bR);

        titanDrive = new MecanumDriveCustom(fL, fR, bL, bR);
        fL.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse the front Motors
        fR.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse the front Motors
        side = AutonomousMode.Side.Red;
        initWebcam(side);
        initHardware();

        PropPosition propDetection = propDetectionPipeline.getPosition();
        telemetry.addData("Position: ", propDetectionPipeline.getPosition().toString());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //webcam.setPipeline(aprilTagDetectionPipeline);
        //Position: 0 = Left, 2 = Right, 1 = Middle

        int pos = 0;
        if (propDetectionPipeline.getPosition() == PropPosition.LEFT) {
            pos = 0;
        }
        if (propDetectionPipeline.getPosition() == PropPosition.RIGHT) {
            pos = 2;
        }
        if (propDetectionPipeline.getPosition() == PropPosition.MIDDLE) {
            pos = 1;
        }
        //webcam.stopStreaming();
        //webcam.stopRecordingPipeline();
        //while (opModeIsActive()) {
            telemetry.addLine("SCORING PURPLE PIXEL");
            telemetry.update();
            //It is in the left
            if (pos == 2) {
                //Put in Drive sequence to Drop Pixel
                //It is in the Right
                //Move arm up to drive
                shoulderMover.setDrivePosition();
                //move forward to line
                moveForwardInches(28, .4);
                strafeInches(-8 ,.4);
                //lower arm to floor
                shoulderMover.moveToPosition(0);
                sleep(sleepTime);
                //Open Big Claw
                theGaber.openLargeClaw();
                //back up to release pixel
                moveForwardInches(-14, .4);
                sleep(sleepTime);
                shoulderMover.setDrivePosition();
                //turn towards board
                theGaber.closeLargeClaw();
                turnInches(-30, .4);
                //raise up arm so it does not drag on ground


                //back up to board
                moveForwardInches(-31,.4);

                strafeInches(-7, .4);
                sleep(sleepTime);

                //raise up arm
                wristMover.raiseWrist();
                shoulderMover.raiseArm();
                sleep(3000);
                theGaber.openLargeClaw();
                theGaber.openSmallClaw();
                sleep(sleepTime);

                theGaber.closeLargeClaw();
                theGaber.closeSmallClaw();
                sleep(sleepTime);

                moveForwardInches(2, .3);
                sleep(sleepTime);
                shoulderMover.setDrivePosition();
                wristMover.lowerWrist();
                sleep(3000);
                turnInches(30, .4);
                sleep(sleepTime);

                moveForwardInches(-24,.4);
                sleep(sleepTime);



            //It is in the left
            } else if (pos == 0) {
                //Starting and Turning
                shoulderMover.setDrivePosition();
                sleep(sleepTime);
                moveForwardInches(32, .4);
                sleep(sleepTime);
                turnInches(-30,.4);
                //Placing Pixel
                sleep(sleepTime);
                moveForwardInches(7, .4);
                 sleep(sleepTime);
                shoulderMover.lowerArm();
                sleep(sleepTime);
                theGaber.openLargeClaw();
                //Going Back to starting Postion
                sleep(sleepTime);
                moveForwardInches(-10, .4);
                sleep(sleepTime);
                theGaber.closeLargeClaw();
                shoulderMover.setDrivePosition();
                sleep(sleepTime);

                ///Plae Pixel on board
                //back up to board
                moveForwardInches(-39,.4);

                strafeInches(-6, .4);
                sleep(sleepTime);

                //raise up arm
                shoulderMover.raiseArm();
                wristMover.raiseWrist();
                sleep(3000);
                theGaber.openLargeClaw();
                theGaber.openSmallClaw();
                sleep(sleepTime);

                theGaber.closeLargeClaw();
                theGaber.closeSmallClaw();
                sleep(sleepTime);

                moveForwardInches(2, .3);
                sleep(sleepTime);
                shoulderMover.lowerArm();
                wristMover.lowerWrist();
                sleep(3000);
                turnInches(30, .4);
                sleep(sleepTime);

                moveForwardInches(-28,.4);
                sleep(sleepTime);

                ///




            } else {
                //It is in the middle
                //Move arm up to drive
                shoulderMover.setDrivePosition();
                //move forward to line
                moveForwardInches(40, .4);
                //lower arm to floor
                shoulderMover.moveToPosition(0);
                sleep(sleepTime);
                //Open Big Claw
                theGaber.openLargeClaw();
                //back up to release pixel
                moveForwardInches(-14, .4);
                shoulderMover.setDrivePosition();
                sleep(sleepTime);
                //turn towards board
                theGaber.closeLargeClaw();
                turnInches(-30, .4);
                //raise up arm so it does not drag on ground
                shoulderMover.setDrivePosition();

                //back up to board
                moveForwardInches(-41,.4);

                strafeInches(-6, .4);
                sleep(sleepTime);

                //raise up arm
                shoulderMover.raiseArm();
                wristMover.raiseWrist();
                sleep(3000);
                theGaber.openLargeClaw();
                theGaber.openSmallClaw();
                sleep(sleepTime);

                theGaber.closeLargeClaw();
                theGaber.closeSmallClaw();
                sleep(sleepTime);

                moveForwardInches(2, .3);
                sleep(sleepTime);
                shoulderMover.setDrivePosition();
                wristMover.lowerWrist();
                sleep(3000);
                turnInches(30, .4);
                sleep(sleepTime);

                moveForwardInches(-28,.4);
                sleep(sleepTime);

            }

        //}
    }

    public void moveForwardInches(double numberOfInches, double power) {
        double totalRevs = 35 * numberOfInches;
        bL.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setTargetPosition((int) (totalRevs));
        bR.setTargetPosition((int) (totalRevs));
        fL.setTargetPosition((int) (totalRevs));
        fR.setTargetPosition((int) (totalRevs));
        fL.setPower(power);
        bL.setPower(power);
        fR.setPower(power);
        bR.setPower(power);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()  && fL.isBusy()) {
            //telemetry.addLine("busy");
            idle();
        }
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turnInches(double numberOfInches, double power) {
        double totalRevs = 35 * numberOfInches;
        bL.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setTargetPosition((int) (totalRevs));
        bR.setTargetPosition((int) (totalRevs));
        fL.setTargetPosition((int) (totalRevs));
        fR.setTargetPosition((int) (totalRevs));
        fL.setPower(power);
        bL.setPower(power);
        fR.setPower(power);
        bR.setPower(power);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && fL.isBusy() ) {
            idle();
        }
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void strafeInches(double NumberOfInches, double power) {
        double totalRevs = 60 * NumberOfInches;
        bL.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setTargetPosition((int) (totalRevs));
        bR.setTargetPosition((int) (totalRevs));
        fL.setTargetPosition((int) (totalRevs));
        fR.setTargetPosition((int) (totalRevs));
        fL.setPower(power);
        bL.setPower(power);
        fR.setPower(power);
        bR.setPower(power);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()  && fL.isBusy() ) {
            idle();
        }
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void resetMotors(DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.resetDeviceConfigurationForOpMode();
        }
    }

    //for meet 1
    public void initHardware() {
        //drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Variables and Drive Motors
        fL = hardwareMap.get(DcMotorEx.class,"leftFront");
        fR = hardwareMap.get(DcMotorEx.class,"rightFront");
        bL = hardwareMap.get(DcMotorEx.class,"leftRear");
        bR = hardwareMap.get(DcMotorEx.class,"rightRear");

        resetMotors(fL, fR, bL, bR);

        titanDrive = new MecanumDriveCustom(fL, fR, bL, bR);

        fL.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse the front Motors
        fR.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse the front Motors

        //implement all that cool arm and claw stuff too
        theGaber = new gaberClaw( hardwareMap.get(Servo.class, "largeClaw"), hardwareMap.get(Servo.class, "smallClaw"));
        theGaber.closeLargeClaw();
        theGaber.closeSmallClaw();

        //bulk loading is crazy fast
        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        try {
            modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        } catch(Exception ignored) {}
    }

    public void initWebcam(AutonomousMode.Side side) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        propDetectionPipeline = new PropDetection(this.telemetry, side == AutonomousMode.Side.Blue);

        //Detect Team Prop
        webcam.setPipeline(propDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        telemetry.setMsTransmissionInterval(50);
        sleep(300);
    }
}
