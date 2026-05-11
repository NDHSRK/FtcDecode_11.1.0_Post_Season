
package org.firstinspires.ftc.teamcode.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.opmodes.FTCAutoDispatch;
import org.firstinspires.ftc.teamcode.common.FTCErrorHandling;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;

@Autonomous(name = "Test Auto termination", group = "Test")
//@Disabled
public class TestAutoTermination extends LinearOpMode {

    private static final String TAG = FTCAutoDispatch.class.getSimpleName();

    // Cause an error by using ValueOf.valueOf("INVALID").
    enum ValueOf {VALID}

    @Override
    public void runOpMode() throws InterruptedException {

        RobotLogCommon.initialize(RobotLogCommon.LogIdentifier.AUTO_LOG, WorkingDirectory.getWorkingDirectory() + RobotConstants.LOG_DIR);

        telemetry.addLine("Enter runOpMode()");
        telemetry.update();
        RobotLogCommon.i(TAG, "Enter runOpMode()");

        try {
            // Case 1: crash before waitForStart().
            //ValueOf vof = ValueOf.valueOf("INVALID");

            // Case 2: the play button is showing we're in waitForStart(), then
            // the driver hits the small stop button.
            // Where does control go? Is runRobot() called; is finally() called?

            // Wait for the game to start (driver presses START).
            telemetry.addLine("In runOpMode() before waitForStart()");
            telemetry.update();
            RobotLogCommon.i(TAG, "In runOpMode() before waitForStart()");

            waitForStart();

            // Call pseudo-autonomous method here.
            runRobot();

        } catch (Exception ex) {
            FTCErrorHandling.handleFtcErrors(ex, TAG, this);
        } finally {
            RobotLogCommon.i(TAG, "In runOpMode() finally");
            RobotLogCommon.closeLog();
        }
    }

    public void runRobot() {
        try {
            // From FTCAuto 11/5/25 runRobot()
            if (!opModeIsActive()) {
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive at the start of runRobot()");

                // Case 2 - continued.

                //throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runRobot()");
                // Case 3: throw new AutonomousRobotException(TAG, "AutonomousRobotException thrown when !opModeIsActive on entry to runRobot()");
                return;
            }

            RobotLogCommon.i(TAG, "runRobot() is running");

            //RobotLogCommon.i(TAG, "Test autonomous timeout");
            //RobotLogCommon.i(TAG, "Test autonomous exit before timeout");
            /*
            int sleepCount = 0;
            while (opModeIsActive()) {
                // Case 4: test timeout
                //sleep(100);

                // Case 5: test exit before timeout.
                sleepCount++;
                if (sleepCount == 10)
                    break;
                sleep(100);
            }
            */

            // Case 6: trigger "stuck in stop"; hit the stop button
            // while the code is not paying attention to opModeIsActive().
            RobotLogCommon.i(TAG, "Loop while *not* testing opModeIsActive()");
            int loopCount = 0;
            while (loopCount < 50000) {
                loopCount++;
                sleep(100);
            }

            // Case 7: how long can you run while not checking for
            // opModeIsActive()?

        } finally {
            if (!opModeIsActive()) {
                RobotLogCommon.i(TAG, "FTCAuto OpMode not active in finally block");
            } else {
                RobotLogCommon.i(TAG, "In FTCAuto finally block");
            }
        }

        RobotLogCommon.i(TAG, "Exiting runRobot");
        telemetry.addLine("Exiting runRobot");
        telemetry.update();
    }

}
