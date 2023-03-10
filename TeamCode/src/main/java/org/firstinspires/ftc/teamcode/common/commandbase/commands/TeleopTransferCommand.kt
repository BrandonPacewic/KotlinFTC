// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.commandbase.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup

import org.firstinspires.ftc.teamcode.common.drive.localization.Localizer
import org.firstinspires.ftc.teamcode.common.hardware.Robot

class TeleopTransferCommand(robot: Robot) : SequentialCommandGroup(
    InstantCommand({
        Localizer.headingOffset = robot.localizer!!.getHeading()
    })
)
