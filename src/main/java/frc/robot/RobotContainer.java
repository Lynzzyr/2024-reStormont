// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kControllers;
import frc.robot.Constants.kDrive;
import frc.robot.generated.TunerConstantsComp;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    // Controllers
    private final CommandXboxController m_primaryController;

    // Subsystems
    private final Drivetrain sys_drivetrain;

    // Commands
    private final Command cmd_drive;

    public RobotContainer() {

        // Controllers
        m_primaryController = new CommandXboxController(kControllers.PRIMARY_CONTROLLER);

        // Subsystems
        sys_drivetrain = TunerConstantsComp.DriveTrain;

        // Commands
        cmd_drive = sys_drivetrain.drive(
            () -> -m_primaryController.getLeftY() * kDrive.MAX_DRIVE_VELOCIY,
            () -> -m_primaryController.getLeftX() * kDrive.MAX_DRIVE_VELOCIY,
            () -> (m_primaryController.getLeftTriggerAxis() - m_primaryController.getRightTriggerAxis()) * kDrive.MAX_TURN_ANGULAR_VELOCITY
        );

        sys_drivetrain.setDefaultCommand(cmd_drive);

        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return null;
    }
}
