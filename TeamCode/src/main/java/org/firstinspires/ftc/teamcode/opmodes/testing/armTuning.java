package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Disabled
@Config
@TeleOp(name="ArmTuning")
public class armTuning extends OpMode {
    private PIDController controller;

    public static double p = 0.00, i = 0, d = 0.000;
    public static double f = 0.00;

    public static int target = 0;

    private final double ticks_in_degree = 280 / 180.0;

    private DcMotorEx arm_motor1;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor1 = hardwareMap.get(DcMotorEx.class, "shoulder1" );
    }

    @Override
    public void loop() {

        controller.setPID(p, i, d);
        int armPos = arm_motor1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor1.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();


    }

}
