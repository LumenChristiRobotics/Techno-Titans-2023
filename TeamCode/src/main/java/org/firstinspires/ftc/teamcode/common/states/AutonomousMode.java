package org.firstinspires.ftc.teamcode.common.states;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.SampleMecanumDrive;
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

public abstract class AutonomousMode extends LinearOpMode {
    public enum Side {
        Blue,
        Red;
    }

    public List<LynxModule> modules;

    public boolean purplePixelScored = false;
    public OpenCvWebcam webcam;
    public PropDetection propDetectionPipeline;
    public AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public gaberClaw claw;
    static final double FEET_PER_METER = 3.28084;

    public double fx = 822.317;
    public double fy = 822.317;
    public double cx = 319.495;
    public double cy = 242.502;

    public double tagsize = 0.166;
    public SampleMecanumDrive drive;

    //CHANGE BASED ON POSITION
    public int LEFT = 2;
    public int MIDDLE = 6;
    public int RIGHT = 16;

    public AprilTagDetection tagOfInterest = null;

    public Side side;

    public AutonomousMode(Side side) {
        this.side = side;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initWebcam(side);
        initHardware();

        PropPosition propDetection = propDetectionPipeline.getPosition();

        waitForStart();
        webcam.setPipeline(aprilTagDetectionPipeline);

        StateMachine machine = getSequence(propDetection);
        while (opModeIsActive()) {
            modules.get(0).clearBulkCache();
            machine.update();
            drive.update();
        }
    }

    public abstract StateMachine getSequence(PropPosition teamPropPos);

    //for meet 1
    public void initHardware() {
        drive = new SampleMecanumDrive(hardwareMap);


        //implement all that cool arm and claw stuff too
        claw = new gaberClaw(hardwareMap.get(Servo.class, "largeClaw"), hardwareMap.get(Servo.class, "smallClaw"));

        DcMotorEx linearSlideMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "LS1");

        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);


        //bulk loading is crazy fast
        modules = hardwareMap.getAll(LynxModule.class);
        modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        try {
            modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        } catch(Exception ignored) {}
    }

    public void initWebcam(Side side) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        propDetectionPipeline = new PropDetection(this.telemetry, side == Side.Blue);

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
