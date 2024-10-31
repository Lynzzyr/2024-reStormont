// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.kControllers;
import frc.robot.Constants.kDrive;
import frc.robot.commands.Jingle;
import frc.robot.generated.TunerConstantsComp;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    // Controllers
    private final CommandXboxController m_primaryController;

    // Subsystems
    private final Drivetrain sys_drivetrain;

    // Commands
    private final Command cmd_drive;
    private final Command cmd_jingle;

    // Shuffleboard
    private final SendableChooser<Command> autoChooser;
    private final ShuffleboardTab sb_autos;

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
        cmd_jingle = new Jingle();

        sys_drivetrain.setDefaultCommand(cmd_drive);

        NamedCommands.registerCommand("Jingle", cmd_jingle);

        // Shuffleboard
        autoChooser = AutoBuilder.buildAutoChooser();
        sb_autos = Shuffleboard.getTab("Auto Paths");
        sb_autos.add("Choose Auto", autoChooser);

        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
