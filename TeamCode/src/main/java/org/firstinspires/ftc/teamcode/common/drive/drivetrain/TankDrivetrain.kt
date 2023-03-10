// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.drivetrain

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose

/**
 * Controls tank drivetrain hardware.
 */
class TankDrivetrain(hardwareMap: HardwareMap) : Drivetrain(hardwareMap) {
    private val frontLeft: DcMotorEx
    private val frontRight: DcMotorEx
    private val backRight: DcMotorEx
    private val backLeft: DcMotorEx
    private val motors: List<DcMotorEx>

    init {
        frontLeft = hardwareMap.get(DcMotorEx::class.java, "motorFrontLeft")
        frontRight = hardwareMap.get(DcMotorEx::class.java, "motorFrontRight")
        backRight = hardwareMap.get(DcMotorEx::class.java, "motorBackRight")
        backLeft = hardwareMap.get(DcMotorEx::class.java, "motorBackLeft")
        motors = listOf(frontLeft, frontRight, backRight, backLeft)

        motors.forEach {
            val motorConfigurationType = it.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            it.motorType = motorConfigurationType
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
    }

    /**
     * Sets the power of the drivetrain motors taking a x, y power percentage.
     */
    override fun setDrivePower(drivePower: Pose) {
        if (drivePower.heading != 0.0) {
            throw InvalidDrivePowerException()
        }

        setMotorPowers(listOf(
            drivePower.x,
            drivePower.y,
            drivePower.x,
            drivePower.y
        ))
    }

    /**
     * Sets the power of the drivetrain motors taking a list of motor powers.
     *
     * Note: The order of the motors goes clockwise starting from the front left 
     * motor.
     */
    override fun setMotorPowers(motorPowers: List<Double>) {
        if (motorPowers.size != 4) {
            throw InvalidMotorCountException()
        }

        for (i in 0..3) {
            motors[i].power = motorPowers[i]
        }
    }

    /**
     * Thrown when the given list of motor powers does not math the count of
     * motors.
     */
    companion object {
        class InvalidMotorCountException : RuntimeException(
            "Input motor powers must be the same length as the number of motors."
        )

        class InvalidDrivePowerException : RuntimeException(
            "Input drive power heading value must be 0, this is a tank drivetrain."
        )
    }
}
