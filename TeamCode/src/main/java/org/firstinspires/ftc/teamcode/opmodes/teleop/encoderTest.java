package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Disabled
@TeleOp(name="testEncoders")
public class encoderTest extends LinearOpMode {
    private   DcMotorEx motor;


    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class,"rightRear");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Right Encoder Posiion: ", motor.getCurrentPosition());
        telemetry.update();

        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("Right Encoder Posiion: ", motor.getCurrentPosition());
            telemetry.update();
        }

    }
}
