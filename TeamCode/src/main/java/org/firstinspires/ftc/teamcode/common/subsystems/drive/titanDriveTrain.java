package org.firstinspires.ftc.teamcode.common.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class titanDriveTrain {

    private DcMotorEx fL, fR, bL, bR;


    public titanDriveTrain(DcMotorEx fL,DcMotorEx fR,DcMotorEx bL,DcMotorEx bR) {
        this.fL = fL;
        this.fR = fR;
        this.bL = bL;
        this.bR = bR;
        this.bL.setDirection(DcMotor.Direction.FORWARD);
        this.fL.setDirection(DcMotor.Direction.FORWARD);
        this.bR.setDirection(DcMotor.Direction.FORWARD);
        this.fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveForwardInches(double numberOfInches, double power) {
        double totalRevs = 35 * numberOfInches;
        this.bL.setDirection(DcMotor.Direction.FORWARD);
        this.fL.setDirection(DcMotor.Direction.FORWARD);
        this.bR.setDirection(DcMotor.Direction.FORWARD);
        this.fR.setDirection(DcMotor.Direction.REVERSE);

        this.bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.bL.setTargetPosition((int) (totalRevs));
        this.bR.setTargetPosition((int) (totalRevs));
        this.fL.setTargetPosition((int) (totalRevs));
        this.fR.setTargetPosition((int) (totalRevs));
        this.fL.setPower(power);
        this.bL.setPower(power);
        this.fR.setPower(power);
        this.bR.setPower(power);
        this.bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (this.fL.isBusy() && this.fR.isBusy()  && this.bL.isBusy() && this.bR.isBusy()) {
            //telemetry.addLine("busy");

        }
        this.bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turnInches(double numberOfInches, double power) {
        double totalRevs = 35 * numberOfInches;
        this.bL.setDirection(DcMotor.Direction.FORWARD);
        this.fL.setDirection(DcMotor.Direction.FORWARD);
        this.bR.setDirection(DcMotor.Direction.REVERSE);
        this.fR.setDirection(DcMotor.Direction.FORWARD);
        this.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.bL.setTargetPosition((int) (totalRevs));
        this.bR.setTargetPosition((int) (totalRevs));
        this.fL.setTargetPosition((int) (totalRevs));
        this.fR.setTargetPosition((int) (totalRevs));
        this.fL.setPower(power);
        this.bL.setPower(power);
        this.fR.setPower(power);
        this.bR.setPower(power);
        this.bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while ( this.fL.isBusy() && this.fR.isBusy() && this.bL.isBusy() && this.bR.isBusy() ) {

        }
        this.bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
