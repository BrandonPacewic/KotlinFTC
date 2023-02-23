// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.hardware

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

import org.firstinspires.ftc.teamcode.common.drive.mecanum.MecanumDrivetrain
import org.firstinspires.ftc.teamcode.common.drive.localization.TwoWheelLocalizer

class Robot(
    hardwareMap: HardwareMap, 
    private val opMode: OpMode = OpMode.TELEOP
) {
    enum class OpMode {
        TELEOP,
        AUTO
    }

    var drive: MecanumDrivetrain

    // Encoders and localizer are only instantiated in autonomous as it
    // costs unnecessary time in teleop as the localizer is not used during
    // driver control.
    var horizontalEncoder: Encoder? = null
    var lateralEncoder: Encoder? = null
    var localizer: TwoWheelLocalizer? = null

    init {
        drive = MecanumDrivetrain(hardwareMap)

        if (opMode == OpMode.AUTO) {
            // TODO: Update encoder names.
            horizontalEncoder = 
                Encoder(hardwareMap.get(DcMotorEx::class.java, "motorFrontLeft"))
            lateralEncoder = 
                Encoder(hardwareMap.get(DcMotorEx::class.java, "motorFrontRight"))

            // TODO: Reverse encoder directions if necessary.

            // localizer = TwoWheelLocalizer()
        }
    }

    /**
     * Resets the localization encoders and the localizer.
     */
    fun reset() {
        if (opMode == OpMode.AUTO) {
            horizontalEncoder!!.reset()
            lateralEncoder!!.reset()
            localizer!!.reset()
        }
    }
}
