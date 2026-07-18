package org.firstinspires.ftc.teamcode.robot.motion;

import android.os.SystemClock;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutoWorker;
import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Threading;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

// 4/10/2026 Copied in to this project from commit a3c55d0
// (3/6/2026) of project FtcDecode_11.1.0_RR_4348.

// Class that starts up a CompletableFuture whose Callable runs
// until it fills the revolver by taking in 3 artifacts or until
// it is interrupted by the driver.
public class IntakeMotion {

    private static final String TAG = IntakeMotion.class.getSimpleName();

    private enum IntakeMotionStates {
        START,
        INTAKE_IN_PROGRESS,
        PAUSE_IN_PROGRESS,
        FINISH,
    }

    private enum IntakeMotionEvents {
        INITIALIZE, INTAKE, PAUSE_INTAKE, EXIT
    }

    // Thread-related.
    private IntakeCallable intakeCallable;
    private CompletableFuture<Integer> intakeFuture;

    private final AtomicBoolean stopIntake = new AtomicBoolean();
    private volatile boolean pauseIntake = false;
    int artifactsToIntake;
    private int artifactsInRevolver = 0;

    public IntakeMotion() {
    }

    // Start up intake as a CompletableFuture.
    // Return the Future so that the caller can manage
    // its completion. Make sure that this can't be called
    // while an intakeFuture is already running.
    public CompletableFuture<Integer> startIntake(int pArtifactsToIntake) {
        if (intakeFuture != null && !intakeFuture.isDone())
            return null;

        RobotLogCommon.d(TAG, "Starting a CompletableFuture for intake");

        artifactsToIntake = pArtifactsToIntake;
        intakeCallable = new IntakeCallable();
        intakeFuture = Threading.launchAsync(intakeCallable);

        stopIntake.set(false);
        pauseIntake = false;
        return intakeFuture;
    }

    // Pause intake until resume() is called.
    // For use during TeleOp reverse intake.
    public void pause() {
        pauseIntake = true;
    }

    public void resume() {
        pauseIntake = false;
    }

    // This will cause the CompletableFuture to exit after
    // it has completed the current operation.
    public void stopIntake() {
        stopIntake.set(true);
    }

    // To be called from the finally block of FTCAuto and TeleOp.
    public void stopIntakeThread() {
        if (intakeFuture == null || intakeFuture.isDone()) // already stopped?
            return;

        intakeCallable.stopThread(); // Force stop now.
        intakeFuture.complete(0);
    }

    // CallableFuture that simulates the intake of Decode
    // artifacts and exits when the revolver is full (by
    // the expiration of a 3-second timer, i.e. 1 second
    // per artifact) or the driver requests a stop.
    // Returns the number of artifacts in the revolver at
    // the time of completion.
    private class IntakeCallable extends AutoWorker<Integer> {
        IntakeCallable() {
            super();
        }

        public Integer call() {
            RobotLogCommon.d(TAG, "Intake future is running");

            // Sanity check.
            if (artifactsToIntake == 0) {
                RobotLogCommon.d(TAG, "Call to intake 0 artifacts");
                return 0;
            }

            // The timer we use in IntelliJ is not supported in Android
            // API level 25 - our maximum.
            // check = Instant.now();
            // secondsElapsed = Duration.between(start, check).toSeconds();

            // Ideally we'd use a stopwatch class and pause the timer when
            // intake is paused - see Apache Commons StopWatch - but let's
            // not load another library just for this.

            ElapsedTime intakeTimer = new ElapsedTime();
            ElapsedTime pauseTimer = new ElapsedTime(); // timer for a single pause
            double totalTimePaused = 0.0;
            int boundary = 1;

            // Initial state and event for the state machine.
            IntakeMotionStates nextState = IntakeMotionStates.START;
            IntakeMotionEvents nextEvent = IntakeMotionEvents.INITIALIZE;

            while (nextEvent != IntakeMotionEvents.EXIT) {
                switch (nextState) {

                    case START: {
                        if (nextEvent != IntakeMotionEvents.INITIALIZE)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        nextState = IntakeMotionStates.INTAKE_IN_PROGRESS;
                        // start timer for total elapsed time including pauses
                        intakeTimer.reset();
                        // Event
                        nextEvent = IntakeMotionEvents.INTAKE;
                        break;
                    }

                    case INTAKE_IN_PROGRESS: {
                        if (nextEvent != IntakeMotionEvents.INTAKE)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        // First check stop conditions.
                        // stop conditions: (stopThreadRequested() or Thread.interrupted() or stopIntake.get())
                        // [stopIntake.get() is true if the driver releases the intake button.]
                        if (stopThreadRequested() || Thread.interrupted() || stopIntake.get()) {
                            nextState = IntakeMotionStates.FINISH;
                            nextEvent = IntakeMotionEvents.EXIT;
                            break;
                        }

                        // Guard: request to pause intake.
                        if (pauseIntake) {
                            nextState = IntakeMotionStates.PAUSE_IN_PROGRESS;
                            // start pause timer
                            pauseTimer.reset();
                            nextEvent = IntakeMotionEvents.PAUSE_INTAKE;
                            break;
                        }

                        // Intake is running
                        if ((int) ((intakeTimer.milliseconds() - totalTimePaused) / 1000) == boundary) {
                            artifactsInRevolver++; // increment on a 1-second boundary
                            if (++boundary > artifactsToIntake) {
                                nextState = IntakeMotionStates.FINISH;
                                nextEvent = IntakeMotionEvents.EXIT;
                                break;
                            }
                        }
                        break;
                    }

                    case PAUSE_IN_PROGRESS: {
                        if (nextEvent != IntakeMotionEvents.PAUSE_INTAKE)
                            throw new AutonomousRobotException(TAG, "Unexpected event " + nextEvent + " for state " + nextState);

                        if (stopThreadRequested() || Thread.interrupted() || stopIntake.get()) {
                            nextState = IntakeMotionStates.FINISH;
                            nextEvent = IntakeMotionEvents.EXIT;
                        } else if (!pauseIntake) {
                            nextState = IntakeMotionStates.INTAKE_IN_PROGRESS;
                            totalTimePaused += pauseTimer.milliseconds();
                            nextEvent = IntakeMotionEvents.INTAKE; // resume intake
                        }
                        break;
                    }
                }

                SystemClock.sleep(25);
            }
            return artifactsInRevolver;
        }
    }
}

