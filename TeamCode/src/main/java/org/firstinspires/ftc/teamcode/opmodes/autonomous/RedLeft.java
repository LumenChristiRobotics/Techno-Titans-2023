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
import org.firstinspires.ftc.teamcode.common.vision.pipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.common.vision.pipelines.PropDetection;
import org.firstinspires.ftc.teamcode.common.vision.util.PropPosition;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name="RedLeft")
public class RedLeft extends LinearOpMode {

    public List<LynxModule> modules;
    public boolean purplePixelScored = false;
    public OpenCvWebcam webcam;
    public PropDetection propDetectionPipeline;
    public AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public gaberClaw theGaber;

    static final double FEET_PER_METER = 3.28084;

    private DcMotor fL, fR, bL, bR;


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

    Integer sleepTime = 250;

    @Override
    public void runOpMode() {
        // Initialize Variables and Drive Motors
        fL = hardwareMap.get(DcMotor.class,"leftFront");
        fR = hardwareMap.get(DcMotor.class,"rightFront");
        bL = hardwareMap.get(DcMotor.class,"leftRear");
        bR = hardwareMap.get(DcMotor.class,"rightRear");

        //Initialize the Arm
        shoulder1 = hardwareMap.get(DcMotorEx.class, "shoulder1");

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        shoulder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
        //while (opModeIsActive()) {
            telemetry.addLine("SCORING PURPLE PIXEL");
            telemetry.update();

            if (pos == 0) {
                //Put in Drive sequence to Drop Pixel
                telemetry.addData("Task: ", "Move Forward to Prop");
                telemetry.update();

                //Driving to mark
                moveForwardInches(28, .4);
                sleep(500);
                strafeInches(10, .4);
                sleep(500);
                //Open Big Claw
                theGaber.openLargeClaw();
                sleep(500);
                moveForwardInches(-10, .4);
                sleep(500);
                //strafeInches(23, .4);
                //sleep(500);
                //theGaber.openSmallClaw();
                //sleep(500);
                //moveForwardInches(-10, .4);
                //sleep(500);
                //theGaber.closeSmallClaw();

                theGaber.closeLargeClaw();
                sleep(500);




            } else if (pos == 2) {

                sleep(sleepTime);
                moveForwardInches(32, .4);
                sleep(sleepTime);
                turnInches(30,.4);
                //Placing Pixel
                sleep(sleepTime);
                moveForwardInches(6 , .4);
                sleep(sleepTime);
                sleep(sleepTime);
                theGaber.openLargeClaw();
                //Going Back to starting Postion
                sleep(sleepTime);
                moveForwardInches(-10, .4);
                sleep(sleepTime);
                sleep(sleepTime);
                turnInches(-30, .4);
                sleep(sleepTime);

            } else {
                //It is in the middle

                moveForwardInches(38, .4);

                //Open Big Claw
                theGaber.openLargeClaw();
                sleep(500);
                moveForwardInches(-10, .4);
                sleep(500);
                //Strafe robot to the stage
                strafeInches(30, .4);
                sleep(500);
                theGaber.openSmallClaw();
                sleep(500);
                moveForwardInches(-10, .4);
                sleep(500);
                //Back up Robot
                theGaber.closeLargeClaw();
                sleep(500);
                theGaber.closeSmallClaw();
                sleep(500);
            }

        //}
    }

    public void moveWrist(Integer position) {
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setTargetPosition(position);

        if (position > 0) {
            elbow.setPower(.3);
        }
        if (position == 0) {
            elbow.setPower(.2);
        }
    }

    public void moveArm(Integer position) {

        shoulder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulder1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder1.setTargetPosition(position); //750 is good to open

        if (position > 0)  {
            shoulder1.setPower(.3);
        }
        if (position == 0)  {
            shoulder1.setPower(.15);
        }

        //while (!isStopRequested() && shoulder1.isBusy() && shoulder2.isBusy() ) {
        //    telemetry.addData("Motor 1 PID Wait: ", shoulder1.getCurrentPosition());
        //    telemetry.update();
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

        while (opModeIsActive()  && fR.isBusy()) {
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
        while (opModeIsActive() && fR.isBusy() ) {
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
        while (opModeIsActive()  && fR.isBusy() && fL.isBusy() ) {
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
        fL = hardwareMap.get(DcMotor.class,"leftFront");
        fR = hardwareMap.get(DcMotor.class,"rightFront");
        bL = hardwareMap.get(DcMotor.class,"leftRear");
        bR = hardwareMap.get(DcMotor.class,"rightRear");

        resetMotors(fL, fR, bL, bR);

        titanDrive = new MecanumDriveCustom(fL, fR, bL, bR);

        fL.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse the front Motors
        fR.setDirection(DcMotorSimple.Direction.REVERSE); //Reverse the front Motors

        //implement all that cool arm and claw stuff too
        theGaber = new gaberClaw( hardwareMap.get(Servo.class, "largeClaw"), hardwareMap.get(Servo.class, "smallClaw"));
        theGaber.closeLargeClaw();
        theGaber.closeSmallClaw();
        //DcMotorEx linearSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "LS1");

        //linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //linearSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //arm = new Arm(linearSlideMotor, 0.0075, 0.01, 0.0001, 0.005, 384.5/360);
        //arm.setTarget(0);

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
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
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
