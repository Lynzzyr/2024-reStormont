package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kDrive;
import frc.robot.Constants.kDrive.kAutoAlign;
import frc.robot.Constants.kDrive.kAutoPathPlanner;
import frc.robot.Constants.kWaypoints;
import frc.robot.generated.TunerConstantsComp;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	// shuffleboard
	private final Field2d m_field;
	private final ShuffleboardTab sb_calibrationTab;

	private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

	public final SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
			.withDeadband(kDrive.MAX_DRIVE_VELOCIY * 0.1).withRotationalDeadband(kDrive.MAX_TURN_ANGULAR_VELOCITY * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
			m_pigeon2.getRotation2d(), m_modulePositions, getRobotPose(),
			VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // TODO validate STDEVs
			VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1)));

	public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);

		this.setDriveMotorsNeutralMode(NeutralModeValue.Brake);
		this.setTurnMotorsNeutralMode(NeutralModeValue.Brake);

		this.configurePathPlanner();

		if (Utils.isSimulation()) {
			startSimThread();
		}

		this.seedFieldRelative();

		// shuffleboard
		m_field = new Field2d();
		sb_calibrationTab = Shuffleboard.getTab("Calibration");
		sb_calibrationTab.add("Field Map", m_field).withPosition(0, 0).withSize(5, 3);

		setDriveMotorInversions();
	}

	private void configurePathPlanner() {
		double driveBaseRadius = 0;
		for (var moduleLocation : m_moduleLocations) {
			driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
		}

		AutoBuilder.configureHolonomic(this::getAutoRobotPose, // Supplier of current robot pose
				this::seedFieldRelative, // Consumer for seeding pose against auto
				this::getCurrentRobotChassisSpeeds, this::driveFromChassisSpeeds, // Consumer of ChassisSpeeds to drive
																					// the robot
				new HolonomicPathFollowerConfig(
						new PIDConstants(kAutoPathPlanner.TRANSLATION_P, kAutoPathPlanner.TRANSLATION_I,
								kAutoPathPlanner.TRANSLATION_D),
						new PIDConstants(kAutoPathPlanner.ROTATION_P, kAutoPathPlanner.ROTATION_I,
								kAutoPathPlanner.ROTATION_D),
							TunerConstantsComp.kSpeedAt12VoltsMps,
						driveBaseRadius, new ReplanningConfig()),
				() -> {
					Optional<Alliance> alliance = DriverStation.getAlliance();
					if (alliance.isPresent())
						return alliance.get() == Alliance.Red;
					return false;
				}, // Change this if the path needs to be flipped on red vs blue
				this); // Subsystem for requirements
	}

	public void driveFromChassisSpeeds(ChassisSpeeds speeds) {
		// SmartDashboard.putNumber("Xspeed", speeds.vxMetersPerSecond);
		// SmartDashboard.putNumber("Yspeed", speeds.vxMetersPerSecond);
		// SmartDashboard.putNumber("deg/s",
		// Math.toDegrees(speeds.omegaRadiansPerSecond));
		this.setControl(autoRequest.withSpeeds(speeds));
	}

	public void setPose(Pose2d pose, boolean isRed) {
		m_poseEstimator.resetPosition(pose.getRotation(), m_modulePositions, pose);
		m_fieldRelativeOffset = Rotation2d.fromDegrees(isRed ? 180.0 : 0.0);
		updateFieldMap();
	}

	public void setPose(Pose2d pose, Rotation2d rotation, boolean isRed) {
		setPose(new Pose2d(pose.getTranslation(), rotation), isRed);
	}

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	public void runRequest(Supplier<SwerveRequest> requestSupplier) {
		this.setControl(requestSupplier.get());
	}

	public ChassisSpeeds getCurrentRobotChassisSpeeds() {
		return m_kinematics.toChassisSpeeds(getState().ModuleStates);
	}

	public void setDriveMotorsNeutralMode(NeutralModeValue mode) {
		for (SwerveModule module : this.Modules) {
			module.getDriveMotor().setNeutralMode(mode);
		}
	}

	public void setTurnMotorsNeutralMode(NeutralModeValue mode) {
		for (SwerveModule module : this.Modules) {
			module.getSteerMotor().setNeutralMode(mode);
		}
	}

	public void setAllMotorsNeutralMode(NeutralModeValue mode) {
		this.setDriveMotorsNeutralMode(mode);
		this.setTurnMotorsNeutralMode(mode);
	}

	public Command drive(Supplier<Double> velocityX, Supplier<Double> velocityY, Supplier<Double> rotationalRate) {
		return this.applyRequest(() -> this.teleopDrive.withVelocityX(velocityX.get()).withVelocityY(velocityY.get())
				.withRotationalRate(rotationalRate.get()));
	}

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}

	public Pose2d getRobotPose() {
		return m_odometry.getEstimatedPosition();
	}

	public Pose2d getAutoRobotPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	public void driveWheelsAt(double speed) {
		for (SwerveModule module : this.Modules) {
			module.getDriveMotor().set(speed);
		}
	}

	/**
	 * Updates Field2d on shuffleboard
	 */
	private void updateFieldMap() {
		m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

		// DEBUG Shuffleboard printouts
		if (kAutoAlign.AUTO_ALIGN_DEBUG) {
			SmartDashboard.putNumber("Field X", m_odometry.getEstimatedPosition().getX());
			SmartDashboard.putNumber("Field Y", m_odometry.getEstimatedPosition().getY());
		}
	}

	/**
	 * Returns list with positions of all swerve modules on robot
	 *
	 * @return Position of all swerve modules
	 */
	public SwerveModulePosition[] getSwerveModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i = 0; i < this.ModuleCount; i++) {
			SwerveModule module = this.Modules[i];
			positions[i] = module.getPosition(true);
		}

		return positions;
	}

	public Rotation2d getWheelPointDirection() {
		double angle = 0;
		for (SwerveModule module : this.Modules) {
			angle += module.getCANcoder().getPosition().getValueAsDouble();
		}
		return new Rotation2d(angle / (double) this.ModuleCount);
	}

	/**
	 * Pathfinds and Navigates the robot to a given position
	 *
	 * @param targetPose
	 *            Target position tso navigate to
	 * @param m_controller
	 *            Primary driver controller
	 */
	public void navigateTo(Pose2d targetPose, CommandXboxController m_controller) {
		PathConstraints constraints = new PathConstraints(kDrive.MAX_DRIVE_VELOCIY, kDrive.MAX_DRIVE_ACCELERATION,
				kDrive.MAX_TURN_ANGULAR_VELOCITY, kDrive.MAX_TURN_ANGULAR_ACCELERATION);
		BooleanSupplier isAPressed = () -> m_controller.a().getAsBoolean();
		Command pathfind = AutoBuilder.pathfindToPose(targetPose, constraints, 0, 0).onlyWhile(isAPressed);
		pathfind.schedule();
	}

	public void setDriveMotorInversions() {

		for (int i = 0; i < this.ModuleCount; i++) {
			SwerveModule module = this.getModule(i);

			if (i % 2 == 0) { // even = left side
				module.getDriveMotor().setInverted(TunerConstantsComp.kInvertLeftSide);
			} else { // odd = right side
				module.getDriveMotor()
						.setInverted(TunerConstantsComp.kInvertRightSide);
			}
		}
	}

	public Pose2d getAmpWaypoint(BooleanSupplier isRed, double offset) {
		return isRed.getAsBoolean()
				? kWaypoints.AMP_ZONE_RED.plus(new Transform2d(offset, 0, new Rotation2d(0)))
				: kWaypoints.AMP_ZONE_BLUE.plus(new Transform2d(offset, 0, new Rotation2d(0)));
	}

	public double getTrapRotation(BooleanSupplier isRed, double index) {
		if (isRed.getAsBoolean()) {
			// System.out.println("returned red");
			if (index == 1) {
				return kAutoAlign.TRAP_POSITION_11;
			} else if (index == 2) {
				return kAutoAlign.TRAP_POSITION_12;
			} else if (index == 3) {
				return kAutoAlign.TRAP_POSITION_13;
			}
		} else {
			// System.out.println("returned blue");
			if (index == 1) {
				return kAutoAlign.TRAP_POSITION_15;
			} else if (index == 2) {
				return kAutoAlign.TRAP_POSITION_16;
			} else if (index == 3) {
				return kAutoAlign.TRAP_POSITION_14;
			}
		}
		return kAutoAlign.TRAP_POSITION_15;
	}

	public void periodic() {
		updateFieldMap();
	}
}