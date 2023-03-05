// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.drivetrain

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PwmControl

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import org.firstinspires.ftc.teamcode.common.control.PIDController
import org.firstinspires.ftc.teamcode.common.hardware.AnalogEncoder

import kotlin.math.abs
import kotlin.math.sign

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
    var rotationalEncoder: AnalogEncoder
) {
    var rotationalPIDController = PIDController(0.0, 0.0, 0.0)
    var kStatic = 0.0
    var kStaticCutoff = 0.02

    var moduleRotation = 0.0
        get() = normalizeRadians(field - Math.PI)
    var targetRotation = 0.0
        get() = normalizeRadians(field - Math.PI)

    /**
     * Stores whether the wheel is flipped or not.
     *
     * This is an optimization as it is never optimal to spin the wheel
     * 180+ degrees to get to the target angle.
     */
    enum class WheelState(val multiplier: Int) {
        NORMAL(1),
        FLIPPED(-1)
    }

    var wheelState = WheelState.NORMAL

    var drivePower: Double
        get() = driveMotor.power
        set(value) {
            driveMotor.power = value * wheelState.multiplier
        }

    init {
        val motorConfigurationType = driveMotor.motorType.clone()
        motorConfigurationType.achieveableMaxRPMFraction = 1.0
        driveMotor.motorType = motorConfigurationType
        driveMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        rotationalServo.pwmRange = PwmControl.PwmRange(500.0, 2500.0, 5000.0)
    }

    /**
     * Updates the module wheel angle.
     */
    fun update() {
        moduleRotation = rotationalEncoder.getCurrentPosition()
        var error = normalizeRadians(targetRotation - moduleRotation)
        val localTargetRotation: Double

        if (abs(error) > Math.PI / 2) {
            localTargetRotation = normalizeRadians(targetRotation - Math.PI)
            wheelState = WheelState.FLIPPED
        } else {
            localTargetRotation = targetRotation
            wheelState = WheelState.NORMAL
        }

        error = normalizeRadians(localTargetRotation - moduleRotation)
        var power = rotationalPIDController.calculate(0.0, error)
        
        if (abs(error) > kStaticCutoff) {
            power += kStatic
        }

        rotationalServo.power = power * sign(power)
    }
}
