// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;


// Command to run the robot at 1/2 power for 1 second in autonomous
public class AutoCommand extends Command {
  CANDriveSubsystem driveSubsystem;
  CANRollerSubsystem rollerSubsystem;
  private Timer timer;
  private double seconds = 4.0;
  private double rollSeconds = 1.0;
  private boolean isDriving = true;

  
    // Constructor. Runs only once when the command is first created.
    public AutoCommand(CANDriveSubsystem driveSubsystem, CANRollerSubsystem rollerSubsystem) {
      // Save parameter for use later and initialize timer object.
      this.driveSubsystem = driveSubsystem;
      this.rollerSubsystem = rollerSubsystem;
    timer = new Timer();

    // Declare subsystems required by this command. This should include any
    // subsystem this command sets and output of
    addRequirements(driveSubsystem, rollerSubsystem);
  }

  // Runs each time the command is scheduled. For this command, we handle starting
  // the timer.
  @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    if (isDriving) {
      driveSubsystem.driveArcade(0.5, 0.0);
    } else {
      rollerSubsystem.runRoller(1.0, 0.0);
    }
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    driveSubsystem.driveArcade(0.0, 0.0);
    rollerSubsystem.runRoller(0.0, 0.0);
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    if (isDriving && timer.get() >= seconds) {
      isDriving = false;
      timer.restart();
    }
    return !isDriving && timer.get() >= rollSeconds;
  }
}
