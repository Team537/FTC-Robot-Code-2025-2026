package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MotorState;
import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.Command;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.Groups.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Commands.WaitCommand;
import org.firstinspires.ftc.teamcode.util.commandsystem.Subsystem;

import java.util.function.Supplier;

public class MotorSubsystem extends Subsystem {
    private final DcMotorEx intakeMotor;

    public MotorSubsystem(HardwareMap hardwareMap, String motorName) {
        super();

        // Initialize motors from hardware map
        intakeMotor = hardwareMap.get(DcMotorEx.class, motorName);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    public Command getFeedCommand(Supplier<MotorState> motorStateSupplier) {
        return new RunCommand(() -> {
            double power = 0;
            switch (motorStateSupplier.get()) {
                case Forward: power = 1; break;
                case Backward: power = -1; break;
            }
            this.intakeMotor.setPower(power);
            TelemetryManager.put("power", power);
        }).withRequirements(this);

    }

    public Command getRunForTime(double seconds, MotorState motorState) {
       return new SequentialCommandGroup(
         new RunCommand(() -> {
            double power = 0;
            switch (motorState) {
                case Forward: power = 1; break;
                case Backward: power = -1; break;
            }
            this.intakeMotor.setPower(power);
            TelemetryManager.put("power", power);
        }).withRequirements(this),
         new WaitCommand(seconds),
         new RunCommand (() -> {
              this.intakeMotor.setPower(0);
         }));
    }
}