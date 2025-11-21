package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.TelemetryManager;
import org.firstinspires.ftc.teamcode.util.commandsystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.geometry.Pose2d;

/**
 * Core Robot class.
 *
 * Lifecycle is split into:
 * - Robot methods: common to all opmodes
 * - Per-opmode methods: specific to PreMatchInit, Auto, or TeleOp
 *
 * Each opmode should call:
 *   5 robot methods + 5 per-mode methods
 * for a total of 10 calls during init/loop phases.
 */
public class Robot {

    // Reference to the RobotContainer which holds subsystems
    private final RobotContainer robotContainer;
    private OpMode opMode;

    public Robot(OpMode opMode) {
        robotContainer = RobotContainer.getInstance(opMode);
        this.opMode = opMode;
    }

    // ===================== Robot Methods =====================

    /** Called once when opmode init phase starts (all opmodes) */
    public void robotInitStart() {
        TelemetryManager.init(opMode.telemetry);
        robotContainer.setupHardware();
    }

    /** Called repeatedly while opmode is in init but not started */
    public void robotInitLoop() {}

    /** Called repeatedly both in init and active phases */
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        TelemetryManager.update();
    }

    /** Called once when opmode becomes active */
    public void robotActiveStart() {}

    /** Called repeatedly while opmode is active */
    public void robotActiveLoop() {}

    // ===================== PreMatchInit Methods =====================

    /** Called once when PreMatchInit opmode init phase starts */
    public void preMatchInitStart() {}

    /** Called repeatedly while PreMatchInit opmode is in init */
    public void preMatchInitLoop() {}

    /** Called repeatedly both in init and active phases */
    public void preMatchPeriodic() {}

    /** Called once when PreMatchInit opmode becomes active */
    public void preMatchActiveStart() {}

    /** Called repeatedly while PreMatchInit opmode is active */
    public void preMatchActiveLoop() {}

    // ===================== Auto Methods =====================

    /** Called once when Auto opmode init phase starts */
    public void autoInitStart() {

        CommandScheduler.getInstance().cancelAll();


    }

    /** Called repeatedly while Auto opmode is in init */
    public void autoInitLoop() {}

    /** Called repeatedly both in init and active phases */
    public void autoPeriodic() {}

    /** Called once when Auto opmode becomes active */
    public void autoActiveStart() {
        // Sets initial position
        robotContainer.driveSubsystem.setRobotPose(new Pose2d(-8.75, -64.875, 0));
        robotContainer.scheduleAuto();
    }

    /** Called repeatedly while Auto opmode is active */
    public void autoActiveLoop() {}

    // ===================== TeleOp Methods =====================

    /** Called once when TeleOp opmode init phase starts */
    public void teleOpInitStart() {
        robotContainer.resetPose();
    }

    /** Called repeatedly while TeleOp opmode is in init */
    public void teleOpInitLoop() {}

    /** Called repeatedly both in init and active phases */
    public void teleOpPeriodic() {}

    /** Called once when TeleOp opmode becomes active */
    public void teleOpActiveStart() {
        robotContainer.scheduleTeleOp();
    }

    /** Called repeatedly while TeleOp opmode is active */
    public void teleOpActiveLoop() {}
}