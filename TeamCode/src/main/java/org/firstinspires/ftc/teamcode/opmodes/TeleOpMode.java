package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="TeleOp", group="Competition")
public class TeleOpMode extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.robotInitStart();
        robot.teleOpInitStart();
    }

    @Override
    public void init_loop() {
        robot.robotInitLoop();
        robot.robotPeriodic();
        robot.teleOpInitLoop();
        robot.teleOpPeriodic();
    }

    @Override
    public void start() {
        robot.robotActiveStart();
        robot.teleOpActiveStart();
    }

    @Override
    public void loop() {
        robot.robotActiveLoop();
        robot.robotPeriodic();
        robot.teleOpActiveLoop();
        robot.teleOpPeriodic();
    }

}