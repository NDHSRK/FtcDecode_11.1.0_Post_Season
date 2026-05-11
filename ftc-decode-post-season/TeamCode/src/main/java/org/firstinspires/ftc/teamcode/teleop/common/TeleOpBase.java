package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

public abstract class TeleOpBase {

    protected final RobotConstants.Alliance alliance;
    protected final LinearOpMode linearOpMode;
    protected final FTCRobot robot;

    public TeleOpBase(RobotConstants.Alliance pAlliance,
                      LinearOpMode pLinearOpMode, FTCRobot pRobot) {
        alliance = pAlliance;
        linearOpMode = pLinearOpMode;
        robot = pRobot;
    }

    public abstract void runTeleOp() throws Exception;

}
