package org.firstinspires.ftc.teamcode.robot.motion;

import android.os.SystemClock;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.ftcdevcommon.AutoWorker;
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
            intakeTimer.reset(); // start timer for total elapsed time including pauses

            boolean transitionToPause = true;
            boolean intakePaused = false;
            ElapsedTime pauseTimer = new ElapsedTime(); // timer for a single pause
            double totalTimePaused = 0.0;
            int boundary = 1;
            while (!stopThreadRequested() && !Thread.interrupted() && !stopIntake.get()) {
                // Check for a requested pause in intake while outtake is in progress.
                // Start a pause timer here to get the time spent in this pause.
                if (pauseIntake) {
                    if (transitionToPause) {
                        transitionToPause = false;
                        intakePaused = true;
                        pauseTimer.reset();
                    }
                    continue;
                }

                // If we've just come out of a pause, add the time spent in the
                // pause to the total.
                if (intakePaused) {
                    intakePaused = false;
                    transitionToPause = true; // for the next time around
                    totalTimePaused += pauseTimer.milliseconds();
                }

                // Check to see if the intake timer (minus time spent in all pauses)
                // has crossed a boundary, e.g. zero to one seconds.
                if ((intakeTimer.milliseconds() - totalTimePaused) / 1000 == boundary) {
                    artifactsInRevolver++; // increment on a 1-second boundary
                    if (++boundary > artifactsToIntake)
                        break;
                }

                SystemClock.sleep(25);
            }

            return artifactsInRevolver;
        }
    }

}

