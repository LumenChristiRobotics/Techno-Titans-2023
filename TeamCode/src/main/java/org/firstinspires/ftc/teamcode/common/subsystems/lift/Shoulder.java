package org.firstinspires.ftc.teamcode.common.subsystems.lift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.stat.inference.BinomialTest;

public class Shoulder {
    private DcMotorEx shoulder1;

    public static double p = 0.009, i = 0.015, d = 0;
    public static double f = 0.001;

    private PIDController controller;
    private final double ticks_in_degree = 2800 / 360;

    public Shoulder(DcMotorEx shoulder1) {
        this.shoulder1 = shoulder1;

        this.shoulder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shoulder1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
    }

    public void moveToPosition(Integer position) {
        this.shoulder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulder1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder1.setTargetPosition(position);
        this.controller.setPID(p, i, d);
        int armPos = this.shoulder1.getCurrentPosition();
        double pid = this.controller.calculate(armPos, position);
        double ff = Math.cos(Math.toRadians(position / ticks_in_degree)) * f;

        double power = pid + ff;

        this.shoulder1.setPower(power * .15);
    }

    public void setDrivePosition() {
        Integer position = 75;
        this.shoulder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulder1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder1.setTargetPosition(position);
        this.controller.setPID(p, i, d);
        int armPos = this.shoulder1.getCurrentPosition();
        double pid = this.controller.calculate(armPos, position);
        double ff = Math.cos(Math.toRadians(position / ticks_in_degree)) * f;

        double power = pid + ff;


        this.shoulder1.setPower(power * .3);


    }
    public void raiseArm() {
        Integer position = 800;
        this.shoulder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulder1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder1.setTargetPosition(position);
        this.controller.setPID(p, i, d);
        int armPos = this.shoulder1.getCurrentPosition();
        double pid = this.controller.calculate(armPos, position);
        double ff = Math.cos(Math.toRadians(position / ticks_in_degree)) * f;

        double power = pid + ff;

        this.shoulder1.setPower(power*.4);

    }

    public void lowerArm() {
        Integer position = 0;
        this.shoulder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulder1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.shoulder1.setTargetPosition(position);
        //this.shoulder1.setVelocity(Math.toRadians(20));
        this.shoulder1.setPower(.1);
    }

}
