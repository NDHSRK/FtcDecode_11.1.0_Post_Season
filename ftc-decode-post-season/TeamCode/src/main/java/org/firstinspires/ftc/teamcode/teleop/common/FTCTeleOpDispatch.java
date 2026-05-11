package org.firstinspires.ftc.teamcode.teleop.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsDecode;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.lang.reflect.Constructor;

public class FTCTeleOpDispatch {
    
    //## The difference between TeleOp and Autonomous ---
    // There is only one FTCAuto class, which is instantiated in
    // FTCAutoDispatch; FTCAuto handles different OpModes via the
    // use of RobotAction.xml. TeleOp doesn't work that way -
    // here we have to know which TeleOp OpMode *class* to
    // instantiate, for example DecodeTeleop or test
    // OpModes that need access to multiple devices in FTCRobot
    // such as AprilTagPIDTestImpl. All of these TeleOp classes
    // inherit from TeleOpBase and the TeleOp classes are
    // instantiated below by reflection.
    public static void runTeleOp(RobotConstants.RunType pRunType,
                                     RobotConstantsDecode.OpMode pOpMode,
                                     RobotConstants.Alliance pAlliance,
                                     LinearOpMode pLinear,
                                     Class<? extends TeleOpBase> pTeleOpClass) throws InterruptedException {
        final String TAG = FTCTeleOpDispatch.class.getSimpleName();
        pLinear.telemetry.setAutoClear(false); // keep our messages on the driver station

        // All TeleOp OpModes need access to the public data fields and methods in LinearOpMode.
        try {
            RobotLogCommon.initialize(RobotLogCommon.LogIdentifier.TELEOP_LOG, WorkingDirectory.getWorkingDirectory() + RobotConstants.LOG_DIR);

            RobotLogCommon.c(TAG, "Preparing to run TeleOp with run type " + pRunType  + " and OpMode " + pOpMode);
            RobotLogCommon.c(TAG, "Constructing FTCRobot with run type " + pRunType);
            FTCRobot robot = new FTCRobot(pLinear, pRunType);

            pLinear.telemetry.addData(TAG, "Waiting for start ...");
            pLinear.telemetry.update();

            // Construction of a TeleOp OpMode should take place
            // before waitForStart() is called. But this means
            // that you can't call isOpModeActive() in any
            // initialization function or thread. See the document
            // StopOpMode.txt.

            // Get the constructor with the required parameters.
            Constructor<? extends TeleOpBase> constructor = pTeleOpClass.getConstructor(RobotConstants.Alliance.class,
                    LinearOpMode.class, FTCRobot.class);

            // Create an instance using the constructor and pass arguments.
            RobotLogCommon.c(TAG, "Found constructor for " + pTeleOpClass);
            TeleOpBase teleOpInstance = constructor.newInstance(pAlliance, pLinear, robot);

            //?? Where does control go if there's a crash in init?
            // The exception message appears on the Driver Station,
            // it may or may not be written to the native FTCAutoLog
            // (not enough time?) but it is always written to the
            // match log, e.g.
            // 12-27 16:34:06.616  1673  2774 E FTCAutoDispatch:  ** FATAL Java Exception ** expected: /BORWARD read: FORWARD (position:END_TAG </FORWARD>@16:23 in java.io.InputStreamReader@d2b7e51)
            pLinear.waitForStart();

            pLinear.telemetry.addData(TAG, "Running ...");
            pLinear.telemetry.update();

            RobotLogCommon.c(TAG, "Running TeleOp");
            teleOpInstance.runTeleOp();
        } catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, pLinear);
        } finally {
            RobotLogCommon.closeLog();
        }
    }

}
