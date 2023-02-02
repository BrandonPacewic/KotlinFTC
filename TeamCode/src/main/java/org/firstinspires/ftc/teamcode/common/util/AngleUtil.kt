// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.util

import kotlin.math.PI

/**
 * Basic utility for working with angles.
 *
 * Adapted from the Roadrunner path following library.
 */
class AngleUtil {
    companion object {
        fun normalize(angle: Double): Double {
            var modifiedAngle = angle % (2 * PI)
            modifiedAngle = (modifiedAngle + 2 * PI) % (2 * PI)
            
            return modifiedAngle
        }

        fun normalizeDelta(angle: Double): Double {
            var modifiedAngle = normalize(angle)

            if (modifiedAngle > PI) {
                modifiedAngle -= 2 * PI
            }
            
            return modifiedAngle
        }
    }
}
