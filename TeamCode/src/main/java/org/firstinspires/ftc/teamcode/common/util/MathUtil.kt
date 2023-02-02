// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.util

import kotlin.math.abs

class MathUtil {
    companion object {
        const val EPSILON = 1e-9

        fun epsilonEquals(a: Double, b: Double): Boolean {
            return abs(a - b) < EPSILON
        }
    }
}
