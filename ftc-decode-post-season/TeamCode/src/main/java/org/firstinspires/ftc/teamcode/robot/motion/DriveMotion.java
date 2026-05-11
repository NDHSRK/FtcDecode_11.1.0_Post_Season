package org.firstinspires.ftc.teamcode.robot.motion;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;
import org.firstinspires.ftc.teamcode.teleop.common.TeleOpParallelDrive;

import java.util.EnumMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class DriveMotion {

    private static final String TAG = DriveMotion.class.getSimpleName();

    private final LinearOpMode linear;
    private final AtomicBoolean useDriveStickRotation = new AtomicBoolean(true);
    private final AtomicReference<Double> externalRotation = new AtomicReference<>(0.0);

    public DriveMotion(LinearOpMode pLinear) {
        linear = pLinear;
    }

    public void useDriveStickRotation() {
        useDriveStickRotation.set(true);
    }

    public void useExternalRotation() { // e.g. AprilTag PID
        useDriveStickRotation.set(false);
    }

    // Return sanitized drive train power values ready for transmission
    // to the robot. An AtomicBoolean controls whether or not this
    // method uses the drive stick for rotation.
    public EnumMap<FTCRobot.MotorId, Double> updateDrivePower(double pPowerFactor) {

        //## The following comes from the sample BasicOmniOpMode_Linear.
        //## The only differences are that we multiply the power by a
        //## fractional value: pPowerFactor, and we use the name "rotation"
        // instead of "yaw".

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -linear.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = linear.gamepad1.left_stick_x;
        double rotation = linear.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = (axial + lateral + rotation) * pPowerFactor;
        double rightFrontPower = (axial - lateral - rotation) * pPowerFactor;
        double leftBackPower = (axial - lateral + rotation) * pPowerFactor;
        double rightBackPower = (axial + lateral - rotation) * pPowerFactor;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

        if (max >= 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        } else if (!useDriveStickRotation.get()) { // if wheel powers are all less that 100% and using external rotation, add external rotation in
            double externalRotationVal = externalRotation.get();
            if (externalRotationVal > 0) {
                //right values are negative as positive rotation means we are rotating clockwise
                max = Math.max(Math.max(leftFrontPower, -rightFrontPower), Math.max(leftBackPower, -rightBackPower));
                max = Math.max(max, 0);
                externalRotationVal = Math.min(externalRotationVal, 1-max);
                leftFrontPower += externalRotationVal;
                rightFrontPower -= externalRotationVal;
                leftBackPower += externalRotationVal;
                rightBackPower -= externalRotationVal;
            } else if (externalRotationVal < 0) {
                //right values are negative as negative rotation means we are rotating ccw
                double min = Math.min(Math.min(leftFrontPower, -rightFrontPower), Math.min(leftBackPower, -rightBackPower));
                min = Math.min(min, 0);
                externalRotationVal = Math.max(externalRotationVal, -1-min);
                leftFrontPower += externalRotationVal;
                rightFrontPower -= externalRotationVal;
                leftBackPower += externalRotationVal;
                rightBackPower -= externalRotationVal;
            }
        }
        // End excerpt from the sample BasicOmniOpMode_Linear.

        EnumMap<FTCRobot.MotorId, Double> powerMap = new EnumMap<>(FTCRobot.MotorId.class);
        powerMap.put(FTCRobot.MotorId.LEFT_FRONT_DRIVE, leftFrontPower);
        powerMap.put(FTCRobot.MotorId.RIGHT_FRONT_DRIVE, rightFrontPower);
        powerMap.put(FTCRobot.MotorId.LEFT_BACK_DRIVE, leftBackPower);
        powerMap.put(FTCRobot.MotorId.RIGHT_BACK_DRIVE, rightBackPower);

        return powerMap;
    }

    public void updateRotationalPower(double pRotationalPower) {
        if (useDriveStickRotation.get()) // you're in the wrong place
            throw new AutonomousRobotException(TAG, "Attempt to set rotational power externally when drive stick rotation is in force");
        externalRotation.set(pRotationalPower);
    }

}