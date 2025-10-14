package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ShooterConfig {

    public String motorName;

    public DcMotorSimple.Direction motorDirection;

    public double wheelCircumference;

    public double encoderPulsePerRevolution;

    public double gearRatio;

    public double maxVelocity;
    public ShooterConfig (String motorName, DcMotorSimple.Direction direction, double wheelCircumference, double encoderPulsePerRevolution, double gearRatio, double maxVelocity) {
        this.motorName = motorName;
        this.motorDirection = direction;
        this.wheelCircumference = wheelCircumference;
        this.encoderPulsePerRevolution = encoderPulsePerRevolution;
        this.gearRatio = gearRatio;
        this.maxVelocity = maxVelocity;
    }


}
