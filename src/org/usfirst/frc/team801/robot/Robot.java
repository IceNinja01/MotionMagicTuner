/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the setSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.  
 *
 * Tweak the PID gains accordingly.
 */
package org.usfirst.frc.team801.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	TalonSRX _talon = new TalonSRX(0);
//	TalonSRX _talon1 = new TalonSRX(14);
//	TalonSRX _talon2 = new TalonSRX(1);
//	TalonSRX _talon3 = new TalonSRX(15);
	XboxController _joy = new XboxController(0);
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	boolean _lastButton1 = false;
	double targetPositionRotations;
	private Preferences prefs;
	private double kf;
	private double kp;
	private double ki;
	private double kd;
	private double maxPosition;
	private double minPosition;
	private boolean _lastButton2;
	private double maxVel;
	private double accel;
	private double rotPerInch;
	private boolean button1;
	private boolean button2;

	public void robotInit() {
		prefs = Preferences.getInstance();
		 /* set closed loop gains in slot0 */
        kf = prefs.getDouble("Kf", 0.2);
        kp = prefs.getDouble("Kp", 0.5);
        ki = prefs.getDouble("Ki", 0.00);
        kd = prefs.getDouble("Kd", 0.2);
        rotPerInch = prefs.getDouble("RotPerInches", 1.0);
    	//Setup for Motion Magic
    	maxVel = prefs.getInt("CrusieVelocity", 25); //Inches/Sec input

    	maxVel *= rotPerInch;
    	maxVel *= (4096)/10; //convert to native units, 
    	accel = prefs.getInt("Acceleration", 25); //Inches/Sec input
    	accel *= rotPerInch; 
    	accel *= 4096/10;; //convert to native units

    	minPosition = prefs.getDouble("MinPosition", 0.00);
    	maxPosition = prefs.getDouble("MaxPosition", 0.0); //Rotations 12.5in per wheel rotation; 7.5 is equal to 1 full rotation
    	minPosition *= rotPerInch*4096;
    	maxPosition *= rotPerInch*4096;	               
		SmartDashboard.putData(Scheduler.getInstance()); //Displaying the Scheduler status
		/* choose the sensor and sensor direction */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);
//		_talon1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
//				Constants.kTimeoutMs);
//		_talon2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
//				Constants.kTimeoutMs);
//		_talon3.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
//				Constants.kTimeoutMs);

		/* choose to ensure sensor is positive when output is positive */
		_talon.setSensorPhase(Constants.kSensorPhase);
//		_talon1.setSensorPhase(Constants.kSensorPhase);
//		_talon2.setSensorPhase(Constants.kSensorPhase);
//		_talon3.setSensorPhase(Constants.kSensorPhase);
		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
		_talon.setInverted(Constants.kMotorInvert);
		_talon.selectProfileSlot(0, 0);
//		_talon1.setInverted(Constants.kMotorInvert);
//		_talon1.selectProfileSlot(0, 0);
//		_talon2.setInverted(Constants.kMotorInvert);
//		_talon2.selectProfileSlot(0, 0);
//		_talon3.setInverted(Constants.kMotorInvert);
//		_talon3.selectProfileSlot(0, 0);
		/* set the peak and nominal outputs, 12V means full */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(11, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-11, Constants.kTimeoutMs);
//		_talon1.configNominalOutputForward(0, Constants.kTimeoutMs);
//		_talon1.configNominalOutputReverse(0, Constants.kTimeoutMs);
//		_talon1.configPeakOutputForward(11, Constants.kTimeoutMs);
//		_talon1.configPeakOutputReverse(-11, Constants.kTimeoutMs);
//		_talon2.configNominalOutputForward(0, Constants.kTimeoutMs);
//		_talon2.configNominalOutputReverse(0, Constants.kTimeoutMs);
//		_talon2.configPeakOutputForward(11, Constants.kTimeoutMs);
//		_talon2.configPeakOutputReverse(-11, Constants.kTimeoutMs);
//		_talon3.configNominalOutputForward(0, Constants.kTimeoutMs);
//		_talon3.configNominalOutputReverse(0, Constants.kTimeoutMs);
//		_talon3.configPeakOutputForward(11, Constants.kTimeoutMs);
//		_talon3.configPeakOutputReverse(-11, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);	
//		_talon1.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);	
//		_talon2.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);	
//		_talon3.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);	
		
    	_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
//    	_talon1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
//    	_talon2.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
//    	_talon3.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
    	
		
	}
	public void disabledInit() {
		_talon.set(ControlMode.PercentOutput, 0);
		button1 = _joy.getRawButton(5); // left top bumper
		button2 = _joy.getRawButton(6); // right top bumper
		/* save button state for on press detect */
		_lastButton1 = button1;
		_lastButton2 = button2;

	}
	public void disabledPeriodic() {
		button1 = _joy.getRawButton(5); // left top bumper
		button2 = _joy.getRawButton(6); // right top bumper
		/* save button state for on press detect */
		_lastButton1 = button1;
		_lastButton2 = button2;
		

	}
	public void teleopInit() {
        /* set closed loop gains in slot0 */
        kf = prefs.getDouble("Kf", 0.2);
        kp = prefs.getDouble("Kp", 0.2);
        ki = prefs.getDouble("Ki", 0.00);
        kd = prefs.getDouble("Kd", 0.2);
        _talon.config_kF(0, kf, 10);
        _talon.config_kP(0, kp, 10);
        _talon.config_kI(0, ki, 10);
        _talon.config_kD(0, kd, 10);
//        _talon1.config_kF(0, kf, 10);
//        _talon1.config_kP(0, kp, 10);
//        _talon1.config_kI(0, ki, 10);
//        _talon1.config_kD(0, kd, 10);
//        _talon2.config_kF(0, kf, 10);
//        _talon2.config_kP(0, kp, 10);
//        _talon2.config_kI(0, ki, 10);
//        _talon2.config_kD(0, kd, 10);
//        _talon3.config_kF(0, kf, 10);
//        _talon3.config_kP(0, kp, 10);
//        _talon3.config_kI(0, ki, 10);
//        _talon3.config_kD(0, kd, 10);
       	//Setup for Motion Magic
    	maxVel = prefs.getDouble("Cruise Velocity", 1.0); //Inches/Sec input
    	maxVel *= (rotPerInch*4096)/10; //convert to native units
    	accel = prefs.getDouble("Acceleration", 1.0); //Inches/Sec input
    	accel *= (rotPerInch*4096)/10; //convert to native units
    	_talon.configMotionCruiseVelocity((int) maxVel, Constants.kTimeoutMs);
    	_talon.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
//    	_talon1.configMotionCruiseVelocity((int) maxVel, Constants.kTimeoutMs);
//    	_talon1.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
//    	_talon2.configMotionCruiseVelocity((int) maxVel, Constants.kTimeoutMs);
//    	_talon2.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
//    	_talon3.configMotionCruiseVelocity((int) maxVel, Constants.kTimeoutMs);
//    	_talon3.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
    	minPosition = prefs.getDouble("Min Position", 0.00);
    	maxPosition = prefs.getDouble("Max Position", 0.0); //Rotations
    	minPosition *= (rotPerInch*4096);
    	maxPosition *= (rotPerInch*4096);
//    	_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); 
//    	_talon1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//    	_talon2.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//    	_talon3.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    	
	}
	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getY();
		/* get gamepad axis */
		double motorOutput = _talon.getMotorOutputPercent();
		button1 = _joy.getRawButton(5); // left top bumper
		button2 = _joy.getRawButton(6); // right top bumper
		/* prepare line to print */
		_sb.append("\tout:");
		/* cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%"); /* perc */

		_sb.append("\tpos:");
		_sb.append(_talon.getSelectedSensorPosition(0));
		_sb.append("u"); /* units */

		/* on button2 press enter closed-loop mode on target position */
		if (_lastButton2 && button2) {
			/* Position mode - button just pressed */
			targetPositionRotations = maxPosition;
			_talon.set(ControlMode.MotionMagic, targetPositionRotations);
//			_talon1.set(ControlMode.MotionMagic, targetPositionRotations);
//			_talon2.set(ControlMode.MotionMagic, targetPositionRotations);
//			_talon3.set(ControlMode.MotionMagic, targetPositionRotations);

		}
		/* on button1 press enter closed-loop mode on target position */
		if (_lastButton1 && button1) {
			/* Position mode - button just pressed */

			/* 10 Rotations * 4096 u/rev in either direction */
			targetPositionRotations = minPosition;
			_talon.set(ControlMode.MotionMagic, targetPositionRotations);
//			_talon1.set(ControlMode.MotionMagic, targetPositionRotations);
//			_talon2.set(ControlMode.MotionMagic, targetPositionRotations);
//			_talon3.set(ControlMode.MotionMagic, targetPositionRotations);
			
		}
//		
//		else {
//			/* Percent voltage mode */
//			_talon.set(ControlMode.PercentOutput, leftYstick);
//		}

		/* if Talon is in position closed-loop, print some more info */
		if (_talon.getControlMode() == ControlMode.MotionMagic) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(0));
			_sb.append("u"); /* units */

			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations);
			_sb.append("u"); /* units */
		}
		/*
		 * print every ten loops, printing too much too fast is generally bad
		 * for performance
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}
		_sb.setLength(0);
		/* save button state for on press detect */
		_lastButton1 = button1;
		_lastButton2 = button2;
		SmartDashboard.putNumber("Error", _talon.getClosedLoopError(0));
		SmartDashboard.putNumber("Position", _talon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Position", _talon.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Current", _talon.getOutputCurrent());
		SmartDashboard.putNumber("Voltage", _talon.getMotorOutputVoltage());
		SmartDashboard.putNumber("k_p", kp);
		SmartDashboard.putNumber("k_f", kf);
		SmartDashboard.putNumber("k_i", ki);
		SmartDashboard.putNumber("k_d", kd);
	 }
}
