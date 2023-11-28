package org.firstinspires.ftc.teamcode.common.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class driveToAprilTagID {
    // Adjust these numbers to suit your robot.
    double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotorEx leftFrontDrive   = null;  //  Used to control the left drive wheel
    private DcMotorEx rightFrontDrive  = null;  //  Used to control the right drive wheel
    private DcMotorEx leftRearDrive   = null;  //  Used to control the left drive wheel
    private DcMotorEx rightRearDrive  = null;  //  Used to control the right drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private Camera webCam;

    public  driveToAprilTagID(DcMotorEx lF, DcMotorEx rF, DcMotorEx lR, DcMotorEx rR, Camera webCam) {
        this.leftFrontDrive = lF;
        this.rightFrontDrive = rF;
        this.leftRearDrive = lR;
        this.rightRearDrive = rR;
        this.webCam = webCam;
    }

    public void setAprilTagId(int Id) {
        this.DESIRED_TAG_ID = Id;
    }

    public  void moveToId() {
        if (DESIRED_TAG_ID != -1) {
            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
            double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
            double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise
            // Initialize the Apriltag Detection process
            initAprilTag();





        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftPower);
        leftRearDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightRearDrive.setPower(rightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(this.webCam.getCameraName())
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }



}
