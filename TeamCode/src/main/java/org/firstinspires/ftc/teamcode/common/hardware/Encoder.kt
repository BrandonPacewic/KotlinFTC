// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx

/**
 * Wraps a motor instance to provide encoder functionality while allowing 
 * separate inversion of the motor and encoder.
 */
class Encoder(private val motor: DcMotorEx) {
    enum class Direction(var multiplier: Int) {
        FORWARD(1),
        REVERSE(-1);
    }

    var direction = Direction.FORWARD

    /**
     * Returns the current position of the encoder.
     */
    fun getCurrentPosition() = motor.currentPosition * direction.multiplier

    /**
     * Resets the encoder position to zero.
     */
    fun reset() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}
