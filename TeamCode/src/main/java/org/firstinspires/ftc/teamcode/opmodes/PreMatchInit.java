package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="Pre-Match Init", group="Competition")
public class PreMatchInit extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.robotInitStart();
        robot.preMatchInitStart();
    }

    @Override
    public void init_loop() {
        robot.robotInitLoop();
        robot.robotPeriodic();
        robot.preMatchInitLoop();
        robot.preMatchPeriodic();
    }

    @Override
    public void start() {
        robot.robotActiveStart();
        robot.preMatchActiveStart();
    }

    @Override
    public void loop() {
        robot.robotActiveLoop();
        robot.robotPeriodic();
        robot.preMatchActiveLoop();
        robot.preMatchPeriodic();
    }

}