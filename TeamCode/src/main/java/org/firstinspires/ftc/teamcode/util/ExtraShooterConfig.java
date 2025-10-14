package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ExtraShooterConfig {

    public String motorName1;

    public String motorName2;

    public DcMotorSimple.Direction motorDirection1;

    public DcMotorSimple.Direction motorDirection2;

    public double wheelCircumference;

    public double encoderPulsePerRevolution;

    public double gearRatio;

    public double maxVelocity;

    public ExtraShooterConfig (String motorName1, String motorName2, DcMotorSimple.Direction motorDirection1, DcMotorSimple.Direction motorDirection2, double wheelCircumference, double encoderPulsePerRevolution, double gearRatio, double maxVelocity) {
        this.motorName1 = motorName1;
        this.motorName2 = motorName2;
        this.motorDirection1 = motorDirection1;
        this.motorDirection2 = motorDirection2;
        this.wheelCircumference = wheelCircumference;
        this.gearRatio = gearRatio;
        this.maxVelocity = maxVelocity;
    }
}
