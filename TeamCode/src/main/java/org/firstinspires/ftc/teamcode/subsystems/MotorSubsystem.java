package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.MotorState;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.WaitCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class MotorSubsystem extends Subsystem {
    private final DcMotorEx motor;

    private boolean inverted = false;

    public MotorSubsystem(HardwareMap hardwareMap, String motorName) {
        super();

        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorInversion(boolean inverted) {
        this.inverted = inverted;
    }

    public Command getManualCommand(Supplier<MotorState> motorStateSupplier) {
        return new RunCommand(() -> {
            double power = 0;
            switch (motorStateSupplier.get()) {
                case Forward: power = 1; break;
                case Backward: power = -1; break;
            }

            if (this.inverted) {
                power *= -1;
            }

            this.motor.setPower(power);
            TelemetryManager.put("power", power);
        }).withRequirements(this);
    }

    public Command getVelocityCommand(Supplier<Double> rpm) {
        return new RunCommand(() -> {
            double radpersec = rpm.get() * 2 * Math.PI / 60;
            if (this.inverted) {
                radpersec *= -1;
            }
            this.motor.setVelocity(radpersec, AngleUnit.RADIANS);
            TelemetryManager.put("rad per second", radpersec);
        }).withRequirements(this);
    }

    public Command getRunForTimeCommand(double seconds, MotorState motorState) {
        return new SequentialCommandGroup(
            new RunCommand(() -> {
                double power = 0;
                switch (motorState) {
                    case Forward: power = 1; break;
                    case Backward: power = -1; break;
                }

                if (this.inverted) {
                    power *= -1;
                }

                this.motor.setPower(power);
                TelemetryManager.put("power", power);
            }).withRequirements(this),
            new WaitCommand(seconds),
            new RunCommand (() -> {
                this.motor.setPower(0);
            })
        );
    }
}