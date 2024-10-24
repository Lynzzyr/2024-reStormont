package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kDrive.kAutoPathPlanner;
import frc.robot.generated.TunerConstantsComp;

/**
 * Drivetrain class that extends Phoenix 6's SwerveDrivetrain template class.
 * This class will be instantiated in TunerConstantsComp as Drivetrain
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {

    // Swerve drive requests
    private final SwerveRequest.ApplyChassisSpeeds reqAuto;
    public final SwerveRequest.FieldCentric reqTeleop;

    // Odometry
    private final SwerveDrivePoseEstimator m_poseEstimator;

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        // Swerve drive requests
        reqAuto = new SwerveRequest.ApplyChassisSpeeds();
        reqTeleop = new SwerveRequest.FieldCentric()
            .withDeadband(kDrive.MAX_DRIVE_VELOCIY * 0.1)
            .withRotationalDeadband(kDrive.MAX_TURN_ANGULAR_VELOCITY * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        // Odometry
        m_poseEstimator = this.m_odometry;
        
        // Swerve setup
        for (SwerveModule module : this.Modules) {
            module.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
            module.getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
        }

        this.seedFieldRelative();

        for (int i = 0; i < ModuleCount; i++) {
            SwerveModule module = this.getModule(i);

            if (i % 2 == 0)
                module.getDriveMotor().setInverted(TunerConstantsComp.kInvertLeftSide);
            else
                module.getDriveMotor().setInverted(TunerConstantsComp.kInvertRightSide);
        }

        // Configure PathPlanner for holonomic auto paths
        double driveBaseRadius = 0;
        for (Translation2d loc : this.m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, loc.getNorm());
        }

        AutoBuilder.configureHolonomic(
            this::getPose2d,
            this::seedFieldRelative,
            this::getChassisSpeeds,
            this::driveFromChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(kAutoPathPlanner.TRANSLATION_P, kAutoPathPlanner.TRANSLATION_I, kAutoPathPlanner.TRANSLATION_D),
                new PIDConstants(kAutoPathPlanner.ROTATION_P, kAutoPathPlanner.ROTATION_I, kAutoPathPlanner.ROTATION_D),
                TunerConstantsComp.kSpeedAt12VoltsMps,
                driveBaseRadius,
                new ReplanningConfig()
            ),
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) return alliance.get() == Alliance.Red;
                return false;
            },
            this
        );
    }

    /**
     * Applies a swerve request
     * @param requestSupplier
     * @return A command that represents the action
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

    /**
     * Generic method to drive the robot using ChassisSpeeds during autonomous
     */
    public void driveFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        this.setControl(reqAuto.withSpeeds(chassisSpeeds));
    }

    /**
     * Generic method for teleop drive
     * @param xVel
     * @param yVel
     * @param rotation
     * @return A drive command
     */
    public Command drive(Supplier<Double> velX, Supplier<Double> velY, Supplier<Double> rotation) {
        return this.applyRequest(
            () -> this.reqTeleop
                .withVelocityX(velX.get())
                .withVelocityY(velY.get())
                .withRotationalRate(rotation.get())
        );
    }

    /**
     * Get current robot Pose2d
     */
    public Pose2d getPose2d() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Get current robot ChassisSpeeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Get current robot odometry
     * @return
     */
    public Pose2d getPoseEstimator() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Update odometry
     */
    public void updatePoseEstimator() {
        m_poseEstimator.update(m_pigeon2.getRotation2d(), m_modulePositions);
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
    }
}