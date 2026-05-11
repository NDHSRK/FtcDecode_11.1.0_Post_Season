package org.firstinspires.ftc.teamcode.robot.device.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.robot.FTCRobot;

import javax.xml.xpath.XPathExpressionException;

// Robot Drive Train
public class TeleOpDriveTrain extends MultiMotorCore {

    public TeleOpDriveTrain(HardwareMap pHardwareMap, XPathAccess pConfigXPath, String pMotorElementName) throws XPathExpressionException {
        super(pHardwareMap, pConfigXPath, pMotorElementName,
                FTCRobot.MotorId.LEFT_FRONT_DRIVE, FTCRobot.MotorId.RIGHT_FRONT_DRIVE,
                FTCRobot.MotorId.LEFT_BACK_DRIVE, FTCRobot.MotorId.RIGHT_BACK_DRIVE);

        setRunModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}