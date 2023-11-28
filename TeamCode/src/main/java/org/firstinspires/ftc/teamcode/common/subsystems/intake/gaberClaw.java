package org.firstinspires.ftc.teamcode.common.subsystems.intake;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

public class gaberClaw {
    private Servo largeClaw;
    private Servo smallClaw;
    public gaberClaw(Servo largeClaw, Servo smallClaw){
        this.largeClaw = largeClaw;
        this.smallClaw = smallClaw;
        largeClaw.setPosition(0);
        smallClaw.setPosition(0);
    }
    public void closeLargeClaw() {
        largeClaw.setPosition(0);
    }
    public void openLargeClaw() {
        largeClaw.setPosition(0.5);
    }
    public void closeSmallClaw() { smallClaw.setPosition(1);}
    public void openSmallClaw() {smallClaw.setPosition(0);}

    public double getLargeClawServoPosition() {
        return largeClaw.getPosition();
    }
    public double getSmallClawServoPosition() {
        return largeClaw.getPosition();
    }
}