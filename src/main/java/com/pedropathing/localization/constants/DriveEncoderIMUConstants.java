package com.pedropathing.localization.constants;

import com.pedropathing.localization.Encoder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * This is the DriveEncoderIMUConstants class. It holds many constants and parameters for the Drive Encoder IMU Localizer.
 * @author Ace - 5321 Tech Heroes
 * @version 1.5, 12/01/2025
 */

public class DriveEncoderIMUConstants {

    /** The number of inches per ticks of the encoder for forward movement
     * @value Default Value: 1 */
    public static double forwardTicksToInches = 1;

    /** The number of inches per ticks of the encoder for lateral movement (strafing)
     * @value Default Value: 1 */
    public static double strafeTicksToInches = 1;

    /** The number of inches per ticks of the encoder for turning
     * @value Default Value: 1 */
    public static double turnTicksToInches = 1;

    public static double robot_Width = 1;
    public static double robot_Length = 1;

    /** The Hardware Map Name of the IMU (built-in IMU will be Port 0, "imu")
     * @value Default Value: "imu" */
    public static String IMU_HardwareMapName = "imu";

    /** The Orientation of the Control Hub (for IMU) on the Robot
     * @value Default Value: new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT) */
    public static RevHubOrientationOnRobot IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

    /** The direction of the left front encoder
     * @value Default Value: Encoder.REVERSE */
    public static double leftFrontEncoderDirection = Encoder.REVERSE;

    /** The direction of the right front encoder
     * @value Default Value: Encoder.FORWARD */
    public static double rightFrontEncoderDirection = Encoder.FORWARD;

    /** The direction of the left rear encoder
     * @value Default Value: Encoder.REVERSE */
    public static double leftRearEncoderDirection = Encoder.REVERSE;

    /** The direction of the right rear encoder
     * @value Default Value: Encoder.FORWARD */
    public static double rightRearEncoderDirection = Encoder.FORWARD;
}
