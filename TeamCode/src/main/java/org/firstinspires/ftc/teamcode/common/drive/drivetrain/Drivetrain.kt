// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.drivetrain

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose

/**
 * Basic setup for all drivetrain classes.
 *
 * Contains a voltage sensor and a method to set the drive power.
 */
abstract class Drivetrain(hardwareMap: HardwareMap) {
    private val voltageSensor: VoltageSensor
    val voltage: Double
        get() = voltageSensor.voltage

    init {
        voltageSensor = hardwareMap.voltageSensor.iterator().next()
    }

    /**
     * Sets the power of the drivetrain motors taking a x, y, and heading power.
     */
    abstract fun setDrivePower(drivePower: Pose)

    /**
     * Sets the power of the drivetrain motors taking a list of motor powers.
     */
    abstract fun setMotorPowers(motorPowers: List<Double>)
}
