package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="Auto", group="Competition")
public class AutoOpMode extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.robotInitStart();
        robot.autoInitStart();
    }

    @Override
    public void init_loop() {
        robot.robotInitLoop();
        robot.robotPeriodic();
        robot.autoInitLoop();
        robot.autoPeriodic();
    }

    @Override
    public void start() {
        robot.robotActiveStart();
        robot.autoActiveStart();
    }

    @Override
    public void loop() {
        robot.robotActiveLoop();
        robot.robotPeriodic();
        robot.autoActiveLoop();
        robot.autoPeriodic();
    }

}