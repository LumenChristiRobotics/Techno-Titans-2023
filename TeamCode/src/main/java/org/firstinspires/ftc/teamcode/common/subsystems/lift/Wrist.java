package org.firstinspires.ftc.teamcode.common.subsystems.lift;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Wrist {
    private DcMotorEx wrist;

    public static double p = 0.028, i = 0.003, d = 0;
    public static double f = 0.07;

    private PIDController controller;
    private final double ticks_in_degree = 1120 / 360;

    public Wrist(DcMotorEx wrist) {
        this.wrist = wrist;

        this.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.wrist.setDirection(DcMotorSimple.Direction.REVERSE);
        controller = new PIDController(p, i, d);
    }

    public void moveToPosition(Integer position) {
        this.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.wrist.setTargetPosition(position);
        this.controller.setPID(p, i, d);
        int armPos = this.wrist.getCurrentPosition();
        double pid = this.controller.calculate(armPos, position);
        double ff = Math.cos(Math.toRadians(position / ticks_in_degree)) * f;

        double power = pid + ff;

        this.wrist.setPower(power);
    }

    public void raiseWrist() {
        Integer position = 100;
        this.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.wrist.setTargetPosition(position);
        this.controller.setPID(p, i, d);
        int armPos = this.wrist.getCurrentPosition();
        double pid = this.controller.calculate(armPos, position);
        double ff = Math.cos(Math.toRadians(position / ticks_in_degree)) * f;

        double power = pid + ff;
        this.wrist.setPower(power * .1);
    }

    public void lowerWrist() {
        Integer position = 0;
        this.wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.wrist.setTargetPosition(position);

        this.wrist.setPower(.05);

    }
}
