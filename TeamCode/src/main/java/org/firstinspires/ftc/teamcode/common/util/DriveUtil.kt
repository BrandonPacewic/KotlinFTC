// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode.common.util

/**
 * Set of helper constants / functions for running roadrunner with drive motor encoders instead of tracking wheels
 * and feedforward velocity control.
 */
class DriveUtil {
    companion object {
        // Drive motor constants.
        private const val ticksPerRevolutions = 537.6
        private const val wheelRadius = 1.8898 // in
        private const val gearRatio = 1.0

        fun driveEncoderTicksToInches(ticks: Double): Double {
            return (ticks / ticksPerRevolutions) * (wheelRadius * 2 * Math.PI) * gearRatio
        }

        fun driveRpmToVelocity(rpm: Double): Double {
            return rpm * gearRatio * 2 * Math.PI * wheelRadius / 60.0
        }

        fun getMotorVelocityF(ticksPerSecond: Double): Double {
            // See https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx.
            return 32767 / ticksPerSecond
        }
    }
}
