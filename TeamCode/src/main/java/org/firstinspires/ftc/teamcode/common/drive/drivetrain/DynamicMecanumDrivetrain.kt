// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.drivetrain

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose

/**
 * Controls mecanum drivetrain hardware. 
 *
 * Allows for dynamic reconfiguration of drive motor names and directions.
 */
class DynamicMecanumDrivetrain(hardwareMap: HardwareMap) : DynamicDrivetrain(hardwareMap) {
    private val frontLeft: DcMotorEx
    private val frontRight: DcMotorEx
    private val backRight: DcMotorEx
    private val backLeft: DcMotorEx
    private val motors: List<DcMotorEx>

    init {
        frontLeft = hardwareMap.get(DcMotorEx::class.java, driveMotors["frontLeft"])
        frontRight = hardwareMap.get(DcMotorEx::class.java, driveMotors["frontRight"])
        backRight = hardwareMap.get(DcMotorEx::class.java, driveMotors["backRight"])
        backLeft = hardwareMap.get(DcMotorEx::class.java, driveMotors["backLeft"])
        motors = listOf(frontLeft, frontRight, backRight, backLeft)

        motors.forEach {
            val motorConfigurationType = it.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            it.motorType = motorConfigurationType
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        frontLeft.direction = driveMotorDirections["frontLeft"]
        frontRight.direction = driveMotorDirections["frontRight"]
        backRight.direction = driveMotorDirections["backRight"]
        backLeft.direction = driveMotorDirections["backLeft"]
    }

    /**
     * Sets the power of the drivetrain motors taking a x, y, and heading power 
     * percentage.
     */
    override fun setDrivePower(drivePower: Pose) {
        setMotorPowers(listOf(
            drivePower.x + drivePower.y + drivePower.heading,
            drivePower.x - drivePower.y - drivePower.heading,
            drivePower.x + drivePower.y - drivePower.heading,
            drivePower.x - drivePower.y + drivePower.heading
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
            throw InvalidMotorCountException
        }

        for (i in 0..3) {
            motors[i].power = motorPowers[i]
        }
    }

    /**
     * Thrown when the given list of motor powers does not math the count of
     * motors.
     */
    companion object InvalidMotorCountException : RuntimeException(
        "Input motor powers must be the same length as the number of motors."
    )
}
