// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose
import org.firstinspires.ftc.teamcode.common.drive.localization.TwoWheelLocalizer

import javax.annotation.concurrent.GuardedBy

/**
 * Robot controller class wrapping all hardware and subsystems.
 */
class Robot(
    hardwareMap: HardwareMap, 
    private val opMode: OpMode = OpMode.TELEOP
) {
    enum class OpMode {
        TELEOP,
        AUTO
    }

    companion object ImuLock {
        @GuardedBy("ImuLock")
        lateinit var imu: IMU
    }

    var imuAngle: Double = 0.0
    private var imuThread: Thread? = null

    var drive: Drivetrain

    // Encoders and localizer are only instantiated in autonomous as it
    // costs unnecessary time in teleop as the localizer is not used during
    // driver control.
    var horizontalEncoder: Encoder? = null
    var lateralEncoder: Encoder? = null
    var localizer: TwoWheelLocalizer? = null

    init {
        drive = MecanumDrivetrain(hardwareMap)

        synchronized(ImuLock) {
            imu = hardwareMap.get(IMU::class.java, "imu")
            val parameters = IMU.Parameters(RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            ))
            imu.initialize(parameters)
        }

        if (opMode == OpMode.AUTO) {
            // TODO: Update encoder names.
            horizontalEncoder = 
                Encoder(hardwareMap.get(DcMotorEx::class.java, "motorFrontLeft"))
            lateralEncoder = 
                Encoder(hardwareMap.get(DcMotorEx::class.java, "motorFrontRight"))

            // TODO: Reverse encoder directions if necessary.

            localizer = TwoWheelLocalizer(
                { horizontalEncoder!!.getCurrentPosition() },
                Pose(-4.0, -4.0, 0.0),
                { lateralEncoder!!.getCurrentPosition() },
                Pose(4.0, 0.0, Math.toRadians(90.0)),
                { getHeading() }
            )
        }
    }

    /**
     * Starts the IMU thread.
     */
    fun startImuThread(opMode: LinearOpMode) {
        imuThread = Thread {
            while (!opMode.isStopRequested && opMode.opModeIsActive()) {
                synchronized(ImuLock) {
                    imuAngle = -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
                }
            }
        }
        imuThread!!.start()
    }

    /**
     * Gets the current heading angle of the robot.
     */
    fun getHeading(): Double {
        synchronized(ImuLock) {
            return imuAngle
        }
    }

    /**
     * Resets the localization of the robot.
     */
    fun reset() {
        imu.resetYaw()

        if (opMode == OpMode.AUTO) {
            horizontalEncoder!!.reset()
            lateralEncoder!!.reset()
            localizer!!.reset()
        }
    }
}
