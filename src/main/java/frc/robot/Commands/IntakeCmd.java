// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSub;

public class IntakeCmd extends Command {
  private final IntakeSub intake;
  XboxController xboxController = new XboxController(0);
  PS5Controller ps5Controller = new PS5Controller(1);
  /** Creates a new IntakeCmd. */
  public IntakeCmd(IntakeSub intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.run(xboxController.getAButton(), xboxController.getXButton(), 0.6);
    intake.run(ps5Controller.getCrossButton(),ps5Controller.getSquareButton(),0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
