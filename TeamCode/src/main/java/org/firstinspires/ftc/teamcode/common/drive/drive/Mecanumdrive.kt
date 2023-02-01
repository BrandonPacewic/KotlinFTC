// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT

package org.firstinspires.ftc.teamcode.common.drive.drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType

import org.firstinspires.ftc.teamcode.common.drive.trajectory.TrajectorySequence
import org.firstinspires.ftc.teamcode.common.drive.trajectory.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.common.drive.trajectory.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.common.util.DriveUtil

/**
 * Mecanum drive implementation using rev hardware.
 *
 * Adapted from the Roadrunner quickstart.
 */
class MecanumDrive : MecanumDrive {
    companion object {
        private val translationPID = PIDCoefficients(2.0, 0.0, 0.0)
        private val headingPID = PIDCoefficients(2.0, 0.0, 0.0)

        private const val lateralMultiplier = 1.0

        private const val vXWeight = 1.0
        private const val vYWeight = 1.0
        private const val omegaWeight = 1.0

        private const val maxAcceleration = 0.0
        private const val maxVelocity = 0.0
        private const val maxAngularAcceleration = 0.0
        private const val maxAngularVelocity = 0.0

        private const val trackWidth = 0.0

        private const val kV = 0.0
        private const val kA = 0.0
        private const val kStatic = 0.0

        private const val runUsingDriveEncoders = false
    }

    private var trajectorySequenceRunner: TrajectorySequenceRunner? = null

    private var velocityConstraint: TrajectoryVelocityConstraint? = null
    private var accelerationConstraint: TrajectoryAccelerationConstraint? = null

    private var follower: TrajectoryFollower? = null

    private var frontLeft: DcMotorEx
    private var frontRight: DcMotorEx
    private var backLeft: DcMotorEx
    private var backRight: DcMotorEx
    private var motors: Array<DcMotorEx>

    private var batteryVoltageSensor: VoltageSensor? = null

    constructor(hardwareMap: HardwareMap) : super (
        kV,
        kA,
        kStatic,
        trackWidth,
        lateralMultiplier
    ) {
        this.follower = HolonomicPIDVAFollower(
            translationPID,
            translationPID,
            headingPID,
            Pose2d(
                0.1,
                0.1,
                Math.toRadians(1.0)
            ),
            1.0
        )

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        this.frontLeft = hardwareMap.get(DcMotorEx::class.java, "motorFrontLeft")
        this.backLeft = hardwareMap.get(DcMotorEx::class.java, "motorBackLeft")
        this.backRight = hardwareMap.get(DcMotorEx::class.java, "motorBackRight")
        this.frontRight = hardwareMap.get(DcMotorEx::class.java, "motorFrontRight")

        this.motors = arrayOf(frontLeft, backLeft, backRight, frontRight)

        for (motor in motors) {
            val motorConfigurationType = motor.getMotorType().clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        if (runUsingDriveEncoders) {

        }
    }

    fun setMode(runMode: DcMotor.RunMode) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        for (motor in motors) {
            motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun setPIDFCoefficients(runMode: DcMotor.RunMode, pidfCoefficients: PIDFCoefficients) {
        val compensatedCoefficient = PIDFCoefficients(
            pidfCoefficients.p,
            pidfCoefficients.i,
            pidfCoefficients.d,
            pidfCoefficients.f * 12.0 / batteryVoltageSensor!!.voltage
        )

        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficient)
        }
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(
            frontLeft.currentPosition.toDouble(),
            backLeft.currentPosition.toDouble(),
            backRight.currentPosition.toDouble(),
            frontRight.currentPosition.toDouble()
        )
    }

    override fun getWheelVelocities(): List<Double> {
        return listOf(
            DriveUtil.driveEncoderTicksToInches(frontLeft.velocity),
            DriveUtil.driveEncoderTicksToInches(backLeft.velocity),
            DriveUtil.driveEncoderTicksToInches(backRight.velocity),
            DriveUtil.driveEncoderTicksToInches(frontRight.velocity)
        )
    }

    override fun setMotorPowers(frontLeft: Double, backLeft: Double, backRight: Double, frontRight: Double) {
        this.frontLeft.power = frontLeft
        this.backLeft.power = backLeft
        this.backRight.power = backRight
        this.frontRight.power = frontRight
    }

    override fun getRawExternalHeading(): Double {
        return 0.0
    }

    override fun getExternalHeadingVelocity(): Double {
        return 0.0
    }

    fun getVelocityConstraint(maxVelocity: Double, maxAngularVelocity: Double, trackWidth: Double): TrajectoryVelocityConstraint {
        return MinVelocityConstraint(
            listOf(
                AngularVelocityConstraint(maxAngularVelocity),
                MecanumVelocityConstraint(maxVelocity, trackWidth)
            )
        )
    }

    fun getAccelerationConstraint(maxAcceleration: Double): TrajectoryAccelerationConstraint {
        return ProfileAccelerationConstraint(maxAcceleration)
    }
}
