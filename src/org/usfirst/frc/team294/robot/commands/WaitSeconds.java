package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WaitSeconds extends Command {

	private double duration;
	
    public WaitSeconds(double duration) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.duration = duration;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (timeSinceInitialized() >= duration);
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}