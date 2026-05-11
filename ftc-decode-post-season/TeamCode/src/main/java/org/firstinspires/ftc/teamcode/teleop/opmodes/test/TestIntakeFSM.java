package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsDecode;
import org.firstinspires.ftc.teamcode.teleop.common.FTCTeleOpDispatch;

@TeleOp(name = "TestIntakeFSM", group = "Competition")
//@Disabled
public class TestIntakeFSM extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        FTCTeleOpDispatch.runTeleOp(RobotConstants.RunType.TELEOP,
                RobotConstantsDecode.OpMode.TELEOP_TEST,
                RobotConstants.Alliance.NONE, this,
                DecodeIntakeFSM.class);
    }
}