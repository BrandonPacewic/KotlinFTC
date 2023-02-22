// Copyright (c) Brandon Pacewic
// SPDX-License-Identifier: MIT WITH FIRST-exception

package org.firstinspires.ftc.teamcode.common.drive.drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
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

import org.firstinspires.ftc.teamcode.common.drive.drive.DriveSignal
import org.firstinspires.ftc.teamcode.common.drive.localization.Localizer
import org.firstinspires.ftc.teamcode.common.drive.localization.ThreeWheelLocalizer
import org.firstinspires.ftc.teamcode.common.drive.kinematics.MecanumKinematics
import org.firstinspires.ftc.teamcode.common.drive.trajectory.TrajectorySequence
import org.firstinspires.ftc.teamcode.common.drive.trajectory.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.common.drive.trajectory.TrajectorySequenceRunner
import org.firstinspires.ftc.teamcode.common.geometry.Pose2d
import org.firstinspires.ftc.teamcode.common.util.DriveUtil

import kotlin.math.abs

/**
 * Mecanum drive implementation using rev hardware.
 *
 * Adapted from the Roadrunner quickstart.
 */
class MecanumDrive(hardwareMap: HardwareMap) {
    companion object {
        private val translationPID = PIDCoefficients(2.0, 0.0, 0.0)
        private val headingPID = PIDCoefficients(2.0, 0.0, 0.0)

        private val motorVelocityPID: PIDFCoefficients? = null

        private const val lateralMultiplier = 1.0

        private const val vXWeight = 0.7
        private const val vYWeight = 0.7
        private const val headingWeight = 0.8

        private const val maxAcceleration = 0.0
        private const val maxVelocity = 0.0
        private const val maxAngularAcceleration = 0.0
        private const val maxAngularVelocity = 0.0

        private const val trackWidth = 0.0

        private const val kV = 0.0
        private const val kA = 0.0
        private const val kStatic = 0.0

        private const val runUsingDriveEncoders = false

        private const val admissableErrorTranslation = 0.1 // in
        private const val admissableErrorHeading = 1.0 // degrees
        private const val admissableErrorTimeout = 0.3 // seconds

        // Tracking wheel data.
        private const val lateralDistance = 14.0 // in
        private const val forwardOffset = 0.0 // in
    }

    private var trajectorySequenceRunner: TrajectorySequenceRunner

    private lateinit var velocityConstraint: TrajectoryVelocityConstraint
    private lateinit var accelerationConstraint: TrajectoryAccelerationConstraint

    private var follower: TrajectoryFollower

    private var frontLeft: DcMotorEx
    private var frontRight: DcMotorEx
    private var backLeft: DcMotorEx
    private var backRight: DcMotorEx
    private var motors: Array<DcMotorEx>

    private var batteryVoltageSensor: VoltageSensor

    private var localizer: Localizer

    var poseEstimate: Pose2d
        get() = localizer.poseEstimate
        set(value) {
            localizer.poseEstimate = value
        }
    var poseVelocity: Pose2d
        get() = localizer.poseVelocity

    var wheelBase: Double
        get() = trackWidth

    init {
        follower = HolonomicPIDVAFollower(
            translationPID,
            translationPID,
            headingPID,
            Pose2d(
                admissableErrorTranslation,
                admissableErrorTranslation,
                Math.toRadians(admissableErrorHeading)
            ),
            admissableErrorTimeout
        )

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        frontLeft = hardwareMap.get(DcMotorEx::class.java, "motorFrontLeft")
        backLeft = hardwareMap.get(DcMotorEx::class.java, "motorBackLeft")
        backRight = hardwareMap.get(DcMotorEx::class.java, "motorBackRight")
        frontRight = hardwareMap.get(DcMotorEx::class.java, "motorFrontRight")
        motors = arrayOf(frontLeft, backLeft, backRight, frontRight)

        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone() as MotorConfigurationType
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        if (runUsingDriveEncoders) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        if (runUsingDriveEncoders) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, motorVelocityPID!!)
        }

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
    
        localizer = ThreeWheelLocalizer(
            listOf(
                Pose2d(0.0, lateralDistance / 2, 0.0), // left encoder
                Pose2d(0.0, -lateralDistance / 2, 0.0), // right encoder
                Pose2d(forwardOffset, 0.0, Math.toRadians(90.0)) // center encoder
            ),
            hardwareMap
        )

        trajectorySequenceRunner = TrajectorySequenceRunner(follower as HolonomicPIDVAFollower, headingPID)
    }

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, false, velocityConstraint!!, accelerationConstraint!!)
    }

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, velocityConstraint!!, accelerationConstraint!!)
    }

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, startHeading, velocityConstraint!!, accelerationConstraint!!)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(
            startPose, 
            velocityConstraint, 
            accelerationConstraint,
            maxAngularVelocity,
            maxAngularAcceleration,
        )
    }

    fun turnAsync(angle: Double) {
        trajectorySequenceRunner!!.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(poseEstimate)
                .turn(angle)
                .build()
        )
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner!!.followTrajectorySequenceAsync(
            trajectorySequenceBuilder(trajectory.start())
                .addTrajectory(trajectory)
                .build()
        )
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = MecanumKinematics.robotToWheelVelocities(
            driveSignal.velocity,
            trackWidth,
            wheelBase
        )

        val accelerations = MecanumKinematics.robotToWheelAccelerations(
            driveSignal.acceleration,
            trackWidth,
            wheelBase
        )

        
    }

    fun getLastError(): Pose2d {
        return trajectorySequenceRunner.lastPoseError
    }

    fun update() {
        updatePoseEstimate()

        val driveSignal = trajectorySequenceRunner.update(this.poseEstimate, this.poseVelocity)
        setDriveSignal(driveSignal)
    }

    fun updatePoseEstimate() {
        localizer.update()
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) {
            update()
        }
    }

    fun isBusy(): Boolean {
        return trajectorySequenceRunner.isBusy()
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
            pidfCoefficients.f * 12.0 / batteryVoltageSensor.voltage
        )

        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficient)
        }
    }

    fun setMotorPowers(power: Double) {
        for (motor in motors) {
            motor.power = power
        }
    }

    fun setMotorPowers(frontLeft: Double, backLeft: Double, backRight: Double, frontRight: Double) {
        this.frontLeft.power = frontLeft
        this.backLeft.power = backLeft
        this.backRight.power = backRight
        this.frontRight.power = frontRight
    }

    fun setMotorPowers(powers: List<Double>) {
        for (i in motors.indices) {
            motors[i].power = powers[i]
        }
    }

    fun setDrivePower(drivePower: Pose2d) {
        val wheelPowers = MecanumKinematics.robotToWheelVelocities(drivePower, wheelBase, trackWidth)
        setMotorPowers(wheelPowers)
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        // Re-normalize the powers according to the weights.
        val denominator = vXWeight * abs(drivePower.x) 
            + vYWeight * abs(drivePower.y) 
            + headingWeight * abs(drivePower.heading)

        val driveVelocity = Pose2d(
            vXWeight * drivePower.x,
            vYWeight * drivePower.y,
            headingWeight * drivePower.heading
        ).div(denominator)

        setDrivePower(driveVelocity)
    }

    fun getWheelPositions(): List<Double> {
        return listOf(
            frontLeft.currentPosition.toDouble(),
            backLeft.currentPosition.toDouble(),
            backRight.currentPosition.toDouble(),
            frontRight.currentPosition.toDouble()
        )
    }

    fun getWheelVelocities(): List<Double> {
        return listOf(
            DriveUtil.driveEncoderTicksToInches(frontLeft.velocity),
            DriveUtil.driveEncoderTicksToInches(backLeft.velocity),
            DriveUtil.driveEncoderTicksToInches(backRight.velocity),
            DriveUtil.driveEncoderTicksToInches(frontRight.velocity)
        )
    }


    fun getExternalHeading(): Double {
        return 0.0
    }

    fun getExternalHeadingVelocity(): Double {
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
