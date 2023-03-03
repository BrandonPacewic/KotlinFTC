// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.swerve

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import org.firstinspires.ftc.teamcode.common.control.PIDController

/**
 * Controls a single coaxial swerve module.
 *
 * This class is set up for a rotational servo that has a built in absolute
 * encoder. If you are using a seperate encoder, you will need to modify this
 * class.
 */
class CoaxialSwerveModule(
    var driveMotor: DcMotorEx,
    var rotationalServo: CRServoImplEx,
) {
    /**
     * Values specific to the hardware controling the rotation of the wheel
     * controled by the module.
     */
    companion object {
        const val wheelRadius = 1.0
        const val gearRatio = 1.0
        const val encoderTicksPerRevolution = 1.0

        
        /**
         * Converts the rotational encoder ticks to inches.
         */
        fun encoderTicksToInches(ticks: Int) =
            wheelRadius * 2 * Math.PI * ticks / ticksPerEncoderRevolution
    }

    var rotationalPIDController = PIDController(0.0, 0.0, 0.0)
    var kStatic = 0.0

    var moduleRotation = 0.0
        get() = normalizeRadians(field - Math.PI)
    var targetRotation = 0.0
        get() = normalizeRadians(field - Math.PI)

    init {
        var motorConfigurationType = driveMotor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = 1.0
        driveMotor.motorType = motorConfigurationType
        driveMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        driveMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        servo.setPwmRange(500, 2500, 5000)
    }
}