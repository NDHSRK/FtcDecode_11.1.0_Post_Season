package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsDecode;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import org.firstinspires.ftc.teamcode.robot.motion.IntakeMotion;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpBase;


import java.io.IOException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeoutException;

public class DecodeIntakeFSM extends TeleOpBase {

    private static final String TAG = DecodeIntakeFSM.class.getSimpleName();

    private enum DecodeTeleOpState {
        START,
        INTAKE_IN_PROGRESS, INTAKE_DONE,
        OUTTAKE_IN_PROGRESS, OUTTAKE_DONE,
        INTAKE_PAUSED_AND_OUTTAKE_IN_PROGRESS,
        LIFTER_IN_PROGRESS,
        FINISH
    }

    private enum DecodeTeleOpEvent {
        GAMEPAD_EVENT, CHECK_LIFTER_DONE, EXIT
    }

    // Intake
    private final FTCButton intakeButton;
    private final IntakeMotion intakeMotion;
    private CompletableFuture<Integer> intakeFuture;
    private int artifactsToIntake = RobotConstantsDecode.MAX_ARTIFACTS_IN_REVOLVER;
    private int artifactsInRevolver = 0;

    private final FTCButton outtakeButton;

    private final FTCButton lifterButton;
    private final FTCButton stopLifterButton;
    private final ElapsedTime lifterTimer;
    private static final long LIFTER_TIMER_DONE = 5; // simulate a five-second lift
    private final FTCButton exitButton;

    public DecodeIntakeFSM(RobotConstants.Alliance pAlliance,
                           LinearOpMode pLinearOpMode, FTCRobot pRobot) {
        super(pAlliance, pLinearOpMode, pRobot);
        RobotLogCommon.c(TAG, "Constructing DecodeTeleOp");

        intakeButton = new FTCButton(() -> linearOpMode.gamepad1.a);
        outtakeButton = new FTCButton(() -> linearOpMode.gamepad1.x);
        lifterButton = new FTCButton(() -> linearOpMode.gamepad1.y);
        stopLifterButton = new FTCButton(() -> linearOpMode.gamepad1.left_bumper);
        exitButton = new FTCButton(() -> linearOpMode.gamepad1.x);

        // Instantiate simulated intake - but start it later.
        intakeMotion = new IntakeMotion();

        // Instantiate the lifter timer - but start it later
        lifterTimer = new ElapsedTime();

        RobotLogCommon.c(TAG, "Finished constructing DecodeTeleOp");
    }

    @Override
    public void runTeleOp() throws Exception {
        try {
            // Safety check against the case where the driver hits the small stop
            // button during waitForStart(). We want to make sure that finally()
            // still runs. From the FTC SDK documentation for opModeIsActive():
            // "If this method returns false after waitForStart() has previously
            // been called, you should break out of any loops and allow the OpMode
            // to exit at its earliest convenience."
            if (!linearOpMode.opModeIsActive()) {
                //## Do *not* do this throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                RobotLogCommon.e(TAG, "OpMode unexpectedly inactive in runTeleOp()");
                return;
            }

            linearOpMode.telemetry.setAutoClear(true);

            // Initialize the starting point for the state machine.
            DecodeTeleOpState nextState = DecodeTeleOpState.START;
            DecodeTeleOpEvent nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;

            // Main loop.
            while (linearOpMode.opModeIsActive() && nextEvent != DecodeTeleOpEvent.EXIT) {

                linearOpMode.telemetry.addLine("To start intake, press and hold A");
                linearOpMode.telemetry.addLine("From state " + DecodeTeleOpState.INTAKE_IN_PROGRESS + " press B to start outtake, Y to start the lifter, or X to exit");
                linearOpMode.telemetry.addLine("From state " + DecodeTeleOpState.OUTTAKE_IN_PROGRESS + " press Y to start the lifter, or X to exit");
                linearOpMode.telemetry.addLine("From state " + DecodeTeleOpState.LIFTER_IN_PROGRESS + " press X to exit");
                linearOpMode.telemetry.addLine();
                linearOpMode.telemetry.addLine("Current state: " + nextState + ", event " + nextEvent);
                linearOpMode.telemetry.addLine("Current event: " + nextEvent);
                linearOpMode.telemetry.update();

                // Updating all button states here is safer even though
                // not all buttons are queried at each step of the process.
                if (nextEvent == DecodeTeleOpEvent.GAMEPAD_EVENT) {
                    intakeButton.update();
                    outtakeButton.update();
                    lifterButton.update();
                    stopLifterButton.update();
                    exitButton.update();
                }

                //!! To be correct, each state should ensure that the current event
                // applies to the state. If a state reacts only to one event then a
                // simple "if (nextEvent != DecodeTeleOpEvent.GET_GAMEPAD_EVENT)" ->
                // state error is good enough. Otherwise a switch/case construct will
                // be required. This goes for the FSM in IntakeMotion also.
                switch (nextState) {
                    case START: {
                        if (nextEvent != DecodeTeleOpEvent.GAMEPAD_EVENT)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        // Guard
                        if (intakeButton.is(FTCButton.State.TAP)) {
                            // Button press to turn intake ON; assume the Revolver is empty.
                            nextState = DecodeTeleOpState.INTAKE_IN_PROGRESS;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            // Action
                            intakeOnAction();
                        }
                        break;
                    }

                    case INTAKE_IN_PROGRESS: {
                        if (nextEvent != DecodeTeleOpEvent.GAMEPAD_EVENT)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        if (intakeButton.is(FTCButton.State.HELD) && intakeFuture.isDone()) {
                            // The driver is still holding the intake button but intake
                            // has completed automatically because the Revolver is full.
                            nextState = DecodeTeleOpState.INTAKE_DONE;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            // Action
                            intakeDoneAction();
                        } else if (intakeButton.is(FTCButton.State.UP)) {
                            // The driver has released the intake button; stop intake.
                            nextState = DecodeTeleOpState.INTAKE_DONE;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            // Action
                            intakeOffAction();
                            intakeDoneAction();
                        } else if (outtakeButton.is(FTCButton.State.TAP)) {
                            // Turn outtake ON while intake is in progress.
                            nextState = DecodeTeleOpState.INTAKE_PAUSED_AND_OUTTAKE_IN_PROGRESS;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            // Action
                            outtakeOnDuringIntakeAction();
                        }
                        break;
                    }

                    // Intake is complete: the revolver is full or intake has
                    // been turned off with 0 - 2 artifacts in the revolver.
                    case INTAKE_DONE: {
                        if (nextEvent != DecodeTeleOpEvent.GAMEPAD_EVENT)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        if (exitButton.is(FTCButton.State.TAP)) {
                            // For the demonstration, exit on a button press.
                            nextEvent = DecodeTeleOpEvent.EXIT;
                        } else if (intakeButton.is(FTCButton.State.TAP) && !revolverIsFull()) {
                            // Button press to turn intake back ON; the revolver must not be full.
                            nextState = DecodeTeleOpState.INTAKE_IN_PROGRESS;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            intakeOnAction();
                        } else if (outtakeButton.is(FTCButton.State.TAP)) {
                            // Turn outtake ON when intake is not running.
                            nextState = DecodeTeleOpState.OUTTAKE_IN_PROGRESS;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            outtakeOnAction();
                        } else if (lifterButton.is(FTCButton.State.DOUBLE_TAP)) {
                            // Start the lifter.
                            nextState = DecodeTeleOpState.LIFTER_IN_PROGRESS;
                            nextEvent = DecodeTeleOpEvent.CHECK_LIFTER_DONE;
                            startLifterAction();
                        }

                        break;
                    }

                    // Outtake when intake is *not* running.
                    case OUTTAKE_IN_PROGRESS: {
                        if (nextEvent != DecodeTeleOpEvent.GAMEPAD_EVENT)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        if (outtakeButton.is(FTCButton.State.UP)) {
                            // The driver cancels outtake by letting go of the button.
                            nextState = DecodeTeleOpState.OUTTAKE_DONE;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            outtakeOffAction();
                        }

                        break;
                    }

                    // Outtake has been turned OFF.
                    // At this point the revolver may contain from 0 to 3 artifacts.
                    case OUTTAKE_DONE: {
                        if (nextEvent != DecodeTeleOpEvent.GAMEPAD_EVENT)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        if (exitButton.is(FTCButton.State.TAP)) {
                            // Allow an immediate exit.
                            nextEvent = DecodeTeleOpEvent.EXIT;
                        } else if (intakeButton.is(FTCButton.State.TAP) && !revolverIsFull()) {
                            // Button press to turn intake back ON; revolver must not be full.
                            nextState = DecodeTeleOpState.INTAKE_IN_PROGRESS;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                        } else if (outtakeButton.is(FTCButton.State.TAP)) {
                            // Button press to turn outtake back ON.
                            nextState = DecodeTeleOpState.OUTTAKE_IN_PROGRESS;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            outtakeOnAction();
                        }

                        break;
                    }

                    // Outtake when intake is paused.
                    case INTAKE_PAUSED_AND_OUTTAKE_IN_PROGRESS: {
                        if (nextEvent != DecodeTeleOpEvent.GAMEPAD_EVENT)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        if (outtakeButton.is(FTCButton.State.UP)) {
                            // Turn outtake off and resume intake.
                            nextState = DecodeTeleOpState.INTAKE_IN_PROGRESS;
                            nextEvent = DecodeTeleOpEvent.GAMEPAD_EVENT;
                            outtakeOffDuringIntakeAction();
                        }

                        break;
                    }

                    // The lifter is running; wait for completion.
                    // Demonstrates the use of an event other than GET_GAMEPAD_EVENT.
                    case LIFTER_IN_PROGRESS: {
                        if (nextEvent != DecodeTeleOpEvent.CHECK_LIFTER_DONE)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        // The driver lets the lifter run to completion.
                        if (checkLiftDone()) {
                            nextEvent = DecodeTeleOpEvent.EXIT;
                            RobotLogCommon.d(TAG, "Lifter is done, exit");
                        }
                        break;
                    }

                }
            }
        } finally {
            RobotLogCommon.d(TAG, "In finally() block");
            intakeMotion.stopIntakeThread();
        }
    }

    // Guard condition methods.
    private boolean revolverIsFull() {
        return artifactsInRevolver == RobotConstantsDecode.MAX_ARTIFACTS_IN_REVOLVER;
    }

    // Action routine methods.
    // Intake
    private void intakeOnAction() {
        /*
        robot.frontIntakeMotor.intake();
        robot.middleIntakeServo.intake();
        robot.backIntakeServo.intake();
        */

        artifactsToIntake -= artifactsInRevolver;
        CompletableFuture<Integer> localIntakeFuture = intakeMotion.startIntake(artifactsToIntake);
        if (localIntakeFuture == null)
            throw new AutonomousRobotException(TAG, "Illegal state: request to intake when intake is already in progress");

        intakeFuture = localIntakeFuture; // intake thread is running
    }

    private void intakeOffAction() {
        intakeMotion.stopIntake();
        RobotLogCommon.d(TAG, "Intake interrupted by driver: intake turned off");
    }

    // If the IntakeFuture completed because the revolver is full
    // then getFutureCompletion() will return immediately. If the
    // driver has toggled the intake button to OFF then
    // getFutureCompletion() will wait here until the IntakeFuture
    // is complete.
    private void intakeDoneAction() throws IOException, InterruptedException, TimeoutException {
        artifactsInRevolver = Threading.getFutureCompletion(intakeFuture);
        RobotLogCommon.d(TAG, "Intake complete with " + artifactsInRevolver + " artifacts in the revolver");

        intakeFuture = null;

        // Turn off intake.
        //robot.frontIntakeMotor.stop();
        //robot.backIntakeServo.stop();
    }

    // Outtake
    private void outtakeOnAction() {
        RobotLogCommon.d(TAG, "Outtake button pressed; intake is not running");
        //robot.frontIntakeMotor.outtake();
        //robot.middleIntakeServo.outtake();
        //robot.backIntakeServo.outtake();
    }

    private void outtakeOffAction() {
        //robot.frontIntakeMotor.stop();
        //robot.backIntakeServo.stop();
        RobotLogCommon.d(TAG, "Outtake cancelled by driver while intake was not running");
    }

    private void outtakeOnDuringIntakeAction() {
        RobotLogCommon.d(TAG, "Outtake button pressed during intake");
        //robot.frontIntakeMotor.outtake();
        //robot.middleIntakeServo.outtake();
        //robot.backIntakeServo.outtake();

        intakeMotion.pause();
        RobotLogCommon.d(TAG, "Pause intake during reversal");
    }

    private void outtakeOffDuringIntakeAction() {
        //robot.frontIntakeMotor.intake();
        //robot.middleIntakeServo.intake();
        //robot.backIntakeServo.intake();

        intakeMotion.resume();
        RobotLogCommon.d(TAG, "Resume intake in forward direction");
    }

    private void startLifterAction() {
        // For the simulation, just run a 5-second timer.
        RobotLogCommon.d(TAG, "Simulated lifter is running");
        lifterTimer.reset();
    }

    // Check whether the lifter simulation is done.
    private boolean checkLiftDone() {
        return lifterTimer.seconds() >= LIFTER_TIMER_DONE;
    }

}