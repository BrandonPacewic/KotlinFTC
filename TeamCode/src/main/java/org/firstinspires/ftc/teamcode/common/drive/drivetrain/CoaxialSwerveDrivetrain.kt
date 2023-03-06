// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.drivetrain

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose
import org.firstinspires.ftc.teamcode.common.hardware.AnalogEncoder
import org.firstinspires.ftc.teamcode.common.hardware.Robot.OpMode

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

/**
 * Controls coaxial swerve drivetrain hardware.
 */
class CoaxialSwerveDrivetrain(
    hardwareMap: HardwareMap, 
    var opMode: OpMode = OpMode.TELEOP
) : Drivetrain(hardwareMap) {
    companion object {
        const val frontLeftEncoderOffset = 0.0
        const val frontRightEncoderOffset = 0.0
        const val backRightEncoderOffset = 0.0
        const val backLeftEncoderOffset = 0.0

        /**
         * Thrown when the number of modules does not match the number of inputs.
         */
        class InvalidModuleCountException : RuntimeException(
            "Input invloving modules must be the same length as the number of modules."
        )
    }

    private val frontLeft: CoaxialSwerveModule
    private val frontRight: CoaxialSwerveModule
    private val backRight: CoaxialSwerveModule
    private val backLeft: CoaxialSwerveModule
    private val modules: List<CoaxialSwerveModule>

    var trackWidth = 0.0
    var wheelBase = 0.0
    val trackRadius = hypot(trackWidth, wheelBase)

    var minDrivePower = 0.07

    init {
        frontLeft = CoaxialSwerveModule(
            hardwareMap.get(DcMotorEx::class.java, "motorFrontLeft"),
            hardwareMap.get(CRServoImplEx::class.java, "servoFrontLeft"),
            AnalogEncoder(
                hardwareMap.get(AnalogInput::class.java, "encoderFrontLeft")
            ).zero(frontLeftEncoderOffset))
        frontRight = CoaxialSwerveModule(
            hardwareMap.get(DcMotorEx::class.java, "motorFrontRight"),
            hardwareMap.get(CRServoImplEx::class.java, "servoFrontRight"),
            AnalogEncoder(
                hardwareMap.get(AnalogInput::class.java, "encoderFrontRight")
            ).zero(frontRightEncoderOffset))
        backRight = CoaxialSwerveModule(
            hardwareMap.get(DcMotorEx::class.java, "motorBackRight"),
            hardwareMap.get(CRServoImplEx::class.java, "servoBackRight"),
            AnalogEncoder(
                hardwareMap.get(AnalogInput::class.java, "encoderBackRight")
            ).zero(backRightEncoderOffset))
        backLeft = CoaxialSwerveModule(
            hardwareMap.get(DcMotorEx::class.java, "motorBackLeft"),
            hardwareMap.get(CRServoImplEx::class.java, "servoBackLeft"),
            AnalogEncoder(
                hardwareMap.get(AnalogInput::class.java, "encoderBackLeft")
            ).zero(backLeftEncoderOffset))

        modules = listOf(frontLeft, frontRight, backRight, backLeft)
    }

    /**
     * Updates each module's wheel angle.
     *
     * IMPORTANT: update() must be called periodically while running a opMode.
     * Without this the wheel angles will not be updated / controlled
     * correctly as it is running a time based PID controller.
     */
    fun update() {
        modules.forEach {
            it.update()
        }
    }

    /**
     * Sets the power of the drivetrain motors taking a x, y, and heading power 
     * percentage.
     */
    override fun setDrivePower(drivePower: Pose) {
        val grid = listOf(
            drivePower.x - drivePower.heading * (trackWidth / wheelBase),
            drivePower.x + drivePower.heading * (trackWidth / wheelBase),
            drivePower.y - drivePower.heading * (wheelBase / trackWidth),
            drivePower.y + drivePower.heading * (wheelBase / trackWidth)
        )

        setMotorPowers(listOf(
            hypot(grid[1], grid[3]),
            hypot(grid[1], grid[2]),
            hypot(grid[0], grid[2]),
            hypot(grid[0], grid[3])
        ))

        setModuleAngles(listOf(
            atan2(grid[1], grid[3]),
            atan2(grid[1], grid[2]),
            atan2(grid[0], grid[2]),
            atan2(grid[0], grid[3])
        ))
    }

    /**
     * Sets the power of the drivetrain modules taking a list of modules powers.
     *
     * Note: The order of the modules goes clockwise starting from the front
     * left modules.
     */
    override fun setMotorPowers(motorPowers: List<Double>) {
        val motorPowers = motorPowers as MutableList<Double>

        if (motorPowers.size != modules.size) {
            throw InvalidModuleCountException()
        }

        val max = motorPowers.maxOrNull() ?: 1.0

        if (abs(max) > 1.0) {
            motorPowers.replaceAll { it / max }
        }

        for (i in 0..3) {
            modules[i].drivePower = abs(motorPowers[i])
        }
    }

    /**
     * Sets the angle of the drivetrain modules taking a list of module angles.
     *
     * Note: The order of the modules goes clockwise starting from the front
     * left module.
     */
    private fun setModuleAngles(moduleAngles: List<Double>) {
        if (moduleAngles.size != modules.size) {
            throw InvalidModuleCountException()
        }

        for (i in 0..3) {
            // Each angle needs to be normalized.
            modules[i].moduleRotation = moduleAngles[i] % (Math.PI * 2)
        }
    }
}
