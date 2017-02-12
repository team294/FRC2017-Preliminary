package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Output Motors, shooter and right
 */
public class Shooter extends Subsystem {

	// Motor Hardware
	private final CANTalon shooterMotor = new CANTalon(RobotMap.shooterMotor);
	private final CANTalon shooterMotor1 = new CANTalon(RobotMap.shooterMotor1);
	private final CANTalon intakeMotor = new CANTalon(RobotMap.intakeMotor);
	DigitalInput jumper = new DigitalInput(RobotMap.jumper);
	double setSpeed;
	boolean error = false;

	public Shooter() {
		super();
		shooterMotor.setVoltageRampRate(24.0);
		shooterMotor1.setVoltageRampRate(24.0);
		intakeMotor.setVoltageRampRate(24.0);
		if (jumper.get() == false) { // jumper in digital 1 will set PIDF values
									// for the second shooter
									//false means the jumper is present
			shooterMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			shooterMotor.configEncoderCodesPerRev(100);
			// shooterMotor.setPID(50.0, 0.2, 0, 40.0, 6000, 50, 0);
			/*
			 * It looks like the feedforward term sets a percent VBUS. It would
			 * therefore be better to multiply f by VBAT/12 to compensate for
			 * battery voltage variation.
			 */
			// shooterMotor.setPID(.100, 0.0, .06, .00845, 6000, 500, 0); //
			// this was for the one motor system

			shooterMotor.setPID(.02, 0, 1, .0088, 500, 500, 0); // two
																		// motor
																		// system
			shooterMotor.reverseSensor(false);
			shooterMotor.reverseOutput(false);
			shooterMotor.changeControlMode(TalonControlMode.Speed);
			shooterMotor1.reverseOutput(false);

		} else {
			shooterMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			shooterMotor.configEncoderCodesPerRev(100);
			// shooterMotor.setPID(50.0, 0.2, 0, 40.0, 6000, 50, 0);
			/*
			 * It looks like the feedforward term sets a percent VBUS. It would
			 * therefore be better to multiply f by VBAT/12 to compensate for
			 * battery voltage variation.
			 */
			// shooterMotor.setPID(.100, 0.0, .06, .00845, 6000, 500, 0); //
			// this was for the one motor system
			shooterMotor.setPID(.1000, 0.00002, 5, .00827, 6000, 500, 0); // two
																		// motor
																		// system
			shooterMotor.reverseSensor(false);
			shooterMotor.reverseOutput(true);
			shooterMotor.changeControlMode(TalonControlMode.Speed);
			shooterMotor1.reverseOutput(false);

		}

		shooterMotor1.changeControlMode(TalonControlMode.Follower);
		shooterMotor1.set(RobotMap.shooterMotor);
		shooterMotor.enableBrakeMode(false);
		shooterMotor1.enableBrakeMode(false);
		shooterMotor.set(0.0);
	}
	
	/**
	 * Sets the shooter motor to a specific speed
	 * 
	 * @param speed
	 *            Double from 6000 to -100 as rpm
	 */
	public void setShooterMotorToSpeed(double speed) {
		shooterMotor.changeControlMode(TalonControlMode.Speed);
		if (speed > 18000.0){
			speed = 18000.0;
		}
		if (speed < -1000.0){
			speed = -1000.0;
		}// Only run reverse to clear a possible jam
		setSpeed = speed;
		shooterMotor.set(speed);
	}
	
	public void periodicSetF(double fInit){
		double currentBatteryVoltage = shooterMotor.getBusVoltage();
		double f = ((12.1/currentBatteryVoltage)*fInit);
		shooterMotor.setF(f);
	}
	
	public void setIntakeSpeed(double voltage){
		intakeMotor.changeControlMode(TalonControlMode.Voltage);
		if(voltage > 12){
			voltage = 12;
		}
		else if (voltage < -4){
			voltage = -4;
		}
		intakeMotor.set(voltage);
	}
	
	public void useVbusControl(double vbus){
		shooterMotor.changeControlMode(TalonControlMode.PercentVbus);
		if (vbus > 1){
			vbus = 1;
		}
		if (vbus < -1){
			vbus = -1;
		}
		shooterMotor.set(-vbus);	
	}
	
	public void setVoltage(double voltage){
		shooterMotor.changeControlMode(TalonControlMode.Voltage);
		if(voltage < -12){
			voltage = -12;
		}if (voltage > 12){
			voltage = 12;
		}
		shooterMotor.set(voltage);
	}
	
	public double getCurrentSpeed(){
		return shooterMotor.getSpeed();
	}
	
	public void logTalonStatus() {
		Robot.log.writeLog(
				"Motor Speed, " + shooterMotor.getSpeed() +
				", Motor Voltage, " + shooterMotor.getOutputVoltage() +
				", Motor Vbus, " + shooterMotor.getBusVoltage() +
				", Motor Current, " + shooterMotor.getOutputCurrent() +
				", Motor error, " +  (shooterMotor.getSpeed() - setSpeed)
		);		
	}
	
	public void brakeTheShooterMotor(double speed) {
		shooterMotor.changeControlMode(TalonControlMode.Voltage);
		shooterMotor.set(0);

	}

	/**
	 * Sets the right motor to a specific speed
	 * 
	 * @param speed
	 *            Double from -1.0 to 1.0 as a percentage of battery voltage
	 */
	/*
	 * public void setRightMotorToSpeed(double speed) { if (speed > 1.0) speed =
	 * 1.0; if (speed < -1.0) speed = -1.0; shooterMotor1.set(speed); }
	 */

	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Shooter Motor Speed", shooterMotor.getSpeed());
		SmartDashboard.putNumber("VBus - Voltage", (shooterMotor.getBusVoltage() - Math.abs(shooterMotor.getOutputVoltage())));
		SmartDashboard.putNumber("Closed Loop Error", shooterMotor.getSpeed() - setSpeed);
		SmartDashboard.putNumber("VBus", shooterMotor.getBusVoltage());
		SmartDashboard.putBoolean("Shooter One", jumper.get());
		SmartDashboard.putNumber("Shooter Motor 1 Current", shooterMotor.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Motor 2 Current", shooterMotor1.getOutputCurrent());
		SmartDashboard.putNumber("Shooter Motor voltage", shooterMotor.getOutputVoltage());
		SmartDashboard.putBoolean("Connection Error", error);
		SmartDashboard.putNumber("Shooter Motor 1000*F", shooterMotor.getF() * 1000);
	}

	public void setupSmartDashboard() {
		//SmartDashboard.putNumber("Shooter Motor 1000*F", shooterMotor.getF() * 1000);
		SmartDashboard.putNumber("Shooter Motor 1000*P", shooterMotor.getP() * 1000);
		SmartDashboard.putNumber("Shooter Motor 1000*I", shooterMotor.getI() * 1000);
		SmartDashboard.putNumber("Shooter Motor 1000*D", shooterMotor.getD() * 1000);
		SmartDashboard.putNumber("Shooter Motor Set Speed", shooterMotor.get());
		SmartDashboard.putNumber("Shooter Motor Set Vbus", 0.0);		
		SmartDashboard.putNumber("Fixed Recovery Voltage", shooterMotor.get());
		SmartDashboard.putNumber("Intake Set Voltage", intakeMotor.get());
		SmartDashboard.putNumber("Set Nominal F Value", 8.8);
	}

	public void setPIDFromSmartDashboard(){
		//shooterMotor.setF(SmartDashboard.getNumber("Shooter Motor 1000*F", 0) / 1000);
		shooterMotor.setP(SmartDashboard.getNumber("Shooter Motor 1000*P", 0) / 1000);
		shooterMotor.setI(SmartDashboard.getNumber("Shooter Motor 1000*I", 0) / 1000);
		shooterMotor.setD(SmartDashboard.getNumber("Shooter Motor 1000*D", 0) / 1000);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}

