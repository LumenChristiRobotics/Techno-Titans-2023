package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Config
@TeleOp(name="WristTuning")
public class wristTuning extends OpMode {
    private PIDController wristController;

    public static double p = 0.028, i = 0.003, d = 0;
    public static double f = 0.07;

    public static int target = 0;

    private final double ticks_in_degree = 1120 / 180.0;

    private DcMotorEx wristMotor;

    @Override
    public void init() {
        wristController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wristMotor = hardwareMap.get(DcMotorEx.class, "elbow" );
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        wristController.setPID(p, i, d);
        int armPos = wristMotor.getCurrentPosition();
        double pid = wristController.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        wristMotor.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();


    }

}
