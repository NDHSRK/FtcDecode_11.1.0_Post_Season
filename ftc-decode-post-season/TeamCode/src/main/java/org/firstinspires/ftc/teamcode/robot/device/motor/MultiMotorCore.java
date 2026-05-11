package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.common.RobotLogCommon;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

import javax.xml.xpath.XPathExpressionException;

public abstract class MultiMotorCore {

    private static final String TAG = MultiMotorCore.class.getSimpleName();

    private final EnumMap<FTCRobot.MotorId, DcMotorEx> motorMap = new EnumMap<>(FTCRobot.MotorId.class);

    private final double clicksPerMotorRev;
    private final double maxVelocity; // clicks per second
    public final double minVelocity; // in relation to maxVelocity, e.g. .2.

    public MultiMotorCore(HardwareMap pHardwareMap, XPathAccess pConfigXPath, String pMotorElementName, FTCRobot.MotorId... pMotorIds) throws XPathExpressionException {
        clicksPerMotorRev = pConfigXPath.getRequiredDouble("multiple_motors" + "/@clicks_per_motor_rev");
        double motorRPM = pConfigXPath.getRequiredDouble("multiple_motors" + "/@rpm");
        maxVelocity = Math.floor((clicksPerMotorRev * motorRPM) / 60); // clicks per second
        minVelocity = pConfigXPath.getRequiredDouble("min_velocity");

        // Get the configuration from RobotConfig.xml.
        RobotLogCommon.c(TAG, "Defining the motors for the " + pMotorElementName);
        RobotLogCommon.c(TAG, "Model " + pConfigXPath.getRequiredText("multiple_motors/@model"));
        RobotLogCommon.c(TAG, "Running at " + maxVelocity + " clicks/second");

        for (FTCRobot.MotorId motorId : pMotorIds) {
            // The motor element name is the lowercase version of the MotorId.
            String oneMotorPath = "multiple_motors/" + motorId.toString().toLowerCase();
            RobotLogCommon.c(TAG, "Configuring motor " + oneMotorPath);

            DcMotorEx oneMotor = pHardwareMap.get(DcMotorEx.class, pConfigXPath.getRequiredText(oneMotorPath + "/@device_name"));
            oneMotor.setDirection(DcMotor.Direction.valueOf(pConfigXPath.getRequiredText(oneMotorPath + "/@direction")));
            motorMap.put(motorId, oneMotor);
        }

        setZeroPowerBrakeAll();
    }

    public double getClicksPerMotorRev() {
        return clicksPerMotorRev;
    }

    public DcMotor.RunMode getRunMode(FTCRobot.MotorId pMotorId) {
        return Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " getRunMode: motor " + pMotorId + " is not in the current configuration").getMode();
    }

    public void setRunModeAll(DcMotor.RunMode pMode) {
        motorMap.forEach((k, v) -> v.setMode(pMode));
    }

    public void setZeroPowerBrakeAll() {
        motorMap.forEach((k, v) -> v.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void setZeroPowerBrakeBehaviorAll(DcMotor.ZeroPowerBehavior pBehavior) {
        motorMap.forEach((k, v) -> v.setZeroPowerBehavior(pBehavior));
    }

    // Forwarding function for DcMotorEx.
    public EnumMap<FTCRobot.MotorId, PIDFCoefficients> getPIDFCoefficientsAll(DcMotor.RunMode pMode) {
        EnumMap<FTCRobot.MotorId, PIDFCoefficients> coefficients = new EnumMap<>(FTCRobot.MotorId.class);

        motorMap.forEach((k, v) -> coefficients.put(k, v.getPIDFCoefficients(pMode)));
        return coefficients;
    }

    // Forwarding function for DcMotorEx.
    public void setPIDFCoefficientsAll(DcMotor.RunMode pMode, PIDFCoefficients pCoefficients) {
        motorMap.forEach((k, v) -> v.setPIDFCoefficients(pMode, pCoefficients));
    }

    // Forwarding function for DcMotorEx.
    public EnumMap<FTCRobot.MotorId, Integer> getTargetPositionToleranceAll() {
        EnumMap<FTCRobot.MotorId, Integer> tolerances = new EnumMap<>(FTCRobot.MotorId.class);
        motorMap.forEach((k, v) -> tolerances.put(k, v.getTargetPositionTolerance()));
        return tolerances;
    }

    public EnumMap<FTCRobot.MotorId, Integer> getCurrentPositionAll() {
        EnumMap<FTCRobot.MotorId, Integer> currentPositions = new EnumMap<>(FTCRobot.MotorId.class);

        motorMap.forEach((k, v) -> currentPositions.put(k, v.getCurrentPosition()));
        return currentPositions;
    }

    public EnumMap<FTCRobot.MotorId, Integer> getTargetPositionAll() {
        EnumMap<FTCRobot.MotorId, Integer> targetPositions = new EnumMap<>(FTCRobot.MotorId.class);

        motorMap.forEach((k, v) -> targetPositions.put(k, v.getTargetPosition()));
        return targetPositions;
    }

    public void setTargetPositionAll(int pPosition) {
        motorMap.forEach((k, v) -> v.setTargetPosition(pPosition));
    }

    // Since the velocity in clicks per second should be the same
    // for all motors, just get the first one.
    public double getClicksPerSecVelocity() {
     List<DcMotorEx> allMotors = new ArrayList<>(motorMap.values());
     return allMotors.get(0).getVelocity();
    }

    // Run one of the multiple motors at the requested velocity.
    public void runAtVelocity(FTCRobot.MotorId pMotorId, double pVelocity) {
        DcMotorEx motor = Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " runAtVelocity: motor " + pMotorId + " is null in the motorMap");
        Objects.requireNonNull(motorMap.get(pMotorId),
                TAG + " setTargetPosition: motor " + pMotorId + " is null in pVelocityMap").setVelocity(pVelocity * maxVelocity);
    }

    public void runAtVelocityAll(double pVelocity) {
        motorMap.forEach((k, v) ->
        {
            v.setVelocity(pVelocity * maxVelocity);
        });
    }

    public void runAtClicksPerSecVelocityAll(double pClicksPerSec) {
        motorMap.forEach((k, v) -> v.setVelocity(pClicksPerSec));
    }

    public boolean allBusy() {
        Optional<Boolean> atLeastOneMotorNotBusy = Optional.of(motorMap.entrySet().stream()
                .anyMatch(entry -> !entry.getValue().isBusy())); // at least one motor is not busy

        return !atLeastOneMotorNotBusy.get();
    }

    // For power, one motor might run at positive power, the other negative.
    public void runAtPowerAll(EnumMap<FTCRobot.MotorId, Double> pPowerMap) {
        pPowerMap.forEach((k, v) ->
        {
            DcMotorEx motor = Objects.requireNonNull(motorMap.get(k),
                    TAG + " runAtVelocity: motor " + k + " is null in the motorMap");
            motor.setPower(v);
        });
    }

    public void stopAllZeroVelocity() {
        RobotLogCommon.d(TAG, "Stop all motors - set velocity to zero");
        motorMap.forEach((k, v) -> v.setVelocity(0.0));
    }

    public void stopAllZeroPower() {
        RobotLogCommon.d(TAG, "Stop all motors - set power to zero");
        motorMap.forEach((k, v) -> v.setVelocity(0.0));
    }

}
