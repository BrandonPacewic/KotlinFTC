// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.swerve

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor

import org.firstinspires.ftc.teamcode.common.hardware.Robot.OpMode

/**
 * Controls coaxial swerve drivetrain hardware.
 */
class CoaxialSwerveDrivetrain(
    hardwareMap: HardwareMap, 
    var opMode: OpMode = OpMode.TELEOP
) {
    private val frontLeft: CoaxialSwerveModule
    private val frontRight: CoaxialSwerveModule
    private val backRight: CoaxialSwerveModule
    private val backLeft: CoaxialSwerveModule
    private val modules: List<CoaxialSwerveModule>

    private val voltageSensor: VoltageSensor
    val voltage: Double
        get() = voltageSensor.voltage

    init {
        modules = listOf(frontLeft, frontRight, backRight, backLeft)

        voltageSensor = hardwareMap.voltageSensor.iterator().next()
    }
}
