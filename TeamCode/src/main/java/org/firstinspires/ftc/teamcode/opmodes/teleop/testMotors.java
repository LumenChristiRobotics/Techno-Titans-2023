package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Disabled
@TeleOp(name="testMotors")
public class testMotors extends LinearOpMode {
    private   DcMotorEx motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class,"rightRear");

        waitForStart();

        motor.setPower(1);
    }
}
