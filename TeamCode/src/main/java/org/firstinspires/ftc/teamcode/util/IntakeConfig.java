package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class IntakeConfig {

    public String motorName;

    public DcMotorSimple.Direction motorDirection;

    public double gearRatio;

    public double wheelCircumference;

    public double encoderPulsePerRevolution;

    public double ticksPerRevolution;

    public double maxVelocity;

    public IntakeConfig (String motorName, DcMotorSimple.Direction motorDirection, double gearRatio, double wheelCircumference, double encoderPulsePerRevolution, double ticksPerRevolution, double maxVelocity) {
        this.motorName = motorName;
        this.motorDirection = motorDirection;
        this.gearRatio = gearRatio;
        this.wheelCircumference = wheelCircumference;
        this.encoderPulsePerRevolution = encoderPulsePerRevolution;
        this.ticksPerRevolution = ticksPerRevolution;
        this.maxVelocity = maxVelocity;
    }
}
