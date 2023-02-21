// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

public class AlignToTargetCommand extends CommandBase {
  DriveSubsystem m_DriveSubsystem;
  ShuffleboardSubsystem m_ShuffleboardSubsystem;

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);

  ProfiledPIDController xPidController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  ProfiledPIDController yPidController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  ProfiledPIDController thetaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  double targetToAlignTo = -1;

  /** Creates a new AlignToTargetCommand. */
  public AlignToTargetCommand(DriveSubsystem p_DriveSubsystem, ShuffleboardSubsystem p_ShuffleboardSubsystem) {
    m_DriveSubsystem = p_DriveSubsystem;
    m_ShuffleboardSubsystem = p_ShuffleboardSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p_DriveSubsystem, p_ShuffleboardSubsystem);

    // Setup PID Controllers
    xPidController.setTolerance(0.2);
    yPidController.setTolerance(0.2);
    thetaController.setTolerance(Units.degreesToRadians(3));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetToAlignTo = table.getValue("tid").getDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetToAlignTo != table.getValue("tid").getDouble()) return;

    xPidController.setGoal(0);
    yPidController.setGoal(0);

    double xSpeed = MathUtil.clamp(xPidController.calculate(table.getValue("ty").getDouble()) / 100, -.5, .5);
    double ySpeed = MathUtil.clamp(yPidController.calculate(table.getValue("tx").getDouble()) / 100, -.5, .5);

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);

    if (xPidController.atGoal()) xSpeed = 0;
    if (yPidController.atGoal()) ySpeed = 0;

    m_DriveSubsystem.drive(xSpeed, ySpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xPidController.atGoal() && yPidController.atGoal();
  }
}
