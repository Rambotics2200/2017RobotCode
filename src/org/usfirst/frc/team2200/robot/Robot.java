package org.usfirst.frc.team2200.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SampleRobot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;


public class Robot extends SampleRobot {
	CANTalon frontRightDriveMotor = new CANTalon(PinsClass.frontRightMotorPin);
	CANTalon frontLeftDriveMotor = new CANTalon(PinsClass.frontLeftMotorPin);
	CANTalon rearRightDriveMotor = new CANTalon(PinsClass.rearRightMotorPin);
	CANTalon rearLeftDriveMotor = new CANTalon(PinsClass.rearLeftMotorPin);
	CANTalon shooterLifterMotor = new  CANTalon(PinsClass.shooterLifterPin);
	CANTalon slaveShooterLifterMotor = new  CANTalon(PinsClass.slaveShooterLifterPin);
	CANTalon intakeMotor = new  CANTalon(PinsClass.intakePin);
	CANTalon slaveIntakeMotor = new  CANTalon(PinsClass.slaveIntakePin);
	CANTalon carouselMotor = new  CANTalon(PinsClass.carouselPin);
	RobotDrive myRobot = new RobotDrive(frontLeftDriveMotor, frontRightDriveMotor);
	
	DoubleSolenoid gearSolenoid = new DoubleSolenoid(PinsClass.gearSola,PinsClass.gearSolb);
	DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PinsClass.intakeSola,PinsClass.intakeSolb);
	
	Encoder leftDriveEnc = new Encoder(PinsClass.driveEncoderLeftA, PinsClass.driveEncoderLeftb);
	Encoder rightDriveEnc = new Encoder(PinsClass.driveEncoderRightA, PinsClass.driveEncoderRightb);
	DigitalOutput leds = new DigitalOutput(PinsClass.leds);
	
	Joystick driverController = new Joystick(1);
	Joystick opController = new Joystick(2);
	
	AHRS ahrs;
	
	double deadZone = 0.25;
	boolean forward = true;
	boolean x = true;
	boolean up = false;
	boolean jam = false;
	double shooterSpeed = 3150;
	boolean disableAdjust = false;
	final String driveForward = "drive forward";
	final String camera = "camera";
	double shotCounter = 0;
	boolean enableShotCounter = true;
	boolean unjam = false;
	double jamTime = 0;
	boolean shooterOn = false;
	double targetAngle;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	public Robot() {
		myRobot.setExpiration(0.1);
	    
	}

	@Override
	public void robotInit() {
		chooser.addDefault("driveForward", driveForward);
		chooser.addObject("camera", camera);
		
		SmartDashboard.putData("Auto modes", chooser);
		
		rearLeftDriveMotor.changeControlMode(TalonControlMode.Follower);
		rearLeftDriveMotor.set(frontLeftDriveMotor.getDeviceID());
		
		rearRightDriveMotor.changeControlMode(TalonControlMode.Follower);
		rearRightDriveMotor.set(frontRightDriveMotor.getDeviceID());
		
		slaveShooterLifterMotor.changeControlMode(TalonControlMode.Follower);
		slaveShooterLifterMotor.set(shooterLifterMotor.getDeviceID());
		
		slaveIntakeMotor.changeControlMode(TalonControlMode.Follower);
		slaveIntakeMotor.set(intakeMotor.getDeviceID());
		
		frontLeftDriveMotor.enableBrakeMode(false);
		frontRightDriveMotor.enableBrakeMode(false);
		rearLeftDriveMotor.enableBrakeMode(false);
		rearRightDriveMotor.enableBrakeMode(false);
		rightDriveEnc.setReverseDirection(true);
		leftDriveEnc.setDistancePerPulse(0.01047); // Distance measured in feet
		rightDriveEnc.setDistancePerPulse(0.01047);
		
		shooterLifterMotor.enableBrakeMode(true);
		slaveShooterLifterMotor.enableBrakeMode(true);
		
		ahrs = new AHRS(SPI.Port.kMXP);
		
		shooterLifterMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		shooterLifterMotor.reverseSensor(false);
		shooterLifterMotor.configNominalOutputVoltage(+0.0f, -0.0f);
		shooterLifterMotor.configPeakOutputVoltage(12.0f, -12.0f);
		shooterLifterMotor.changeControlMode(TalonControlMode.Speed);
		shooterLifterMotor.setProfile(0);
		shooterLifterMotor.setF(0.0265);
		shooterLifterMotor.setP(0.061);
		shooterLifterMotor.setD(0.01);
		shooterLifterMotor.setIZone(600);
/*		SmartDashboard.putNumber("F", 0.029);
		SmartDashboard.putNumber("ShooterP", 0.09);
		SmartDashboard.putNumber("ShooterI", 0.0001);
		SmartDashboard.putNumber("ShooterD", 5);
		SmartDashboard.putNumber("ShooterRPM", 3150);
		SmartDashboard.putNumber("HopperSpeed", 0.6);*/
		
	}
	
	public void camera(){
		Robot2 two = new Robot2(myRobot);
		two.followCamera();
	}
	
	public void driveDistance(double dist){
		double leftPGain;
		double rightPGain;
		double setPoint = dist;
		while(leftDriveEnc.getDistance() <= setPoint*0.95 && rightDriveEnc.getDistance() <= setPoint*0.95){
			if(leftDriveEnc.getDistance() <= setPoint*0.95){
				leftPGain = (1*((setPoint-leftDriveEnc.getDistance())/setPoint)*SmartDashboard.getNumber("left Drive P value", 0.25));

				if(leftPGain <= deadZone){
					frontLeftDriveMotor.set(deadZone);
				}
				else{
					frontLeftDriveMotor.set(leftPGain);
				}
			}
			else{
				frontLeftDriveMotor.set(0);
			}
			if(rightDriveEnc.getDistance() <= setPoint*0.95){
				rightPGain = (1*((setPoint-rightDriveEnc.getDistance())/setPoint)*SmartDashboard.getNumber("right Drive  P Value", 0.25));
				if(rightPGain <=deadZone){
					frontRightDriveMotor.set(-deadZone);	
				}
				else{
					frontRightDriveMotor.set(-rightPGain);
				}
			}
			else{
				frontRightDriveMotor.set(0);
			}
			SmartDashboard.putNumber("left drive distance", leftDriveEnc.getDistance());
			SmartDashboard.putNumber("right drive distance", rightDriveEnc.getDistance());
			SmartDashboard.putNumber("left Error",((setPoint-leftDriveEnc.getDistance())/setPoint)*100);
			SmartDashboard.putNumber("right Error",((setPoint-rightDriveEnc.getDistance())/setPoint)*100);
		}
	}
	
	/*public double angle(){
		double targetAngle = 45;
		double currentAngle = ahrs.getAngle();
		if (currentAngle > 0){
			currentAngle =  currentAngle*-1;
		}
		else{
			
		}
		double diff = targetAngle - currentAngle;
		SmartDashboard.putNumber("Differeecne",diff);
		return diff;
		
	}*/
	public void turn(double targetAngle){
		frontLeftDriveMotor.enableBrakeMode(true);
		frontRightDriveMotor.enableBrakeMode(true);
		rearLeftDriveMotor.enableBrakeMode(true);
		rearRightDriveMotor.enableBrakeMode(true);
		//double targetAngle = 45;
		double currentAngle = ahrs.getPitch();
		while (targetAngle > currentAngle){
			SmartDashboard.putNumber("Angle", ahrs.getPitch());
			currentAngle = ahrs.getPitch(); //getAngle to get Pitch
			frontLeftDriveMotor.set(-0.3);
			frontRightDriveMotor.set(-0.3);
			Timer.delay(0.005);
		}
		frontLeftDriveMotor.set(0);
		frontRightDriveMotor.set(0);
	}
	
	
	@Override
	public void autonomous() {
		if (chooser.getSelected() == driveForward){
			leftDriveEnc.reset();
			rightDriveEnc.reset();
			ahrs.reset();
			//driveDistance(6.85);
			turn(45);
		}
		else if (chooser.getSelected() == camera){
			camera();
		}
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	
	public void updateSmartDashboard(){
		double i = shooterLifterMotor.getClosedLoopError();
		if (i>1500){
			i=1500;
		}
		i= i*0.146484375;
		if(i>100){
			if(enableShotCounter == true){
				shotCounter = shotCounter+1;
				enableShotCounter = false;
			}			
		}
		else{
			enableShotCounter = true;
		}
		SmartDashboard.putNumber("Target Shooter Speed", shooterSpeed);
		SmartDashboard.putNumber("shooter Enc", shooterLifterMotor.getSpeed());
		SmartDashboard.putNumber("Shooter PID Error", i);	
		SmartDashboard.putNumber("carousel current", carouselMotor.getOutputCurrent());	
		SmartDashboard.putNumber("Shot Counter", shotCounter);
	/*		if(driverController.getRawButton(10)){
			shooterLifterMotor.setP(SmartDashboard.getNumber("ShooterP",0.09));
			shooterLifterMotor.setI(SmartDashboard.getNumber("ShooterI",0.0001));
			shooterLifterMotor.setD(SmartDashboard.getNumber("ShooterD",5));
			shooterLifterMotor.setF(SmartDashboard.getNumber("F", 0.029));
		}*/
}
	
	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(false);
		try{
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(320, 240);
		}
		catch(Exception e){
			
		}
		while (isOperatorControl() && isEnabled()) {
			
				
			updateSmartDashboard();
			
			//Drive
			if(driverController.getRawButton(5)|| forward == true){
				myRobot.arcadeDrive(driverController.getRawAxis(1), driverController.getRawAxis(1));
				forward = true;
			}
			
			//Invert drive for when you drive towards yourself
			if(driverController.getRawButton(6) || forward == false){
				myRobot.arcadeDrive((-(driverController.getRawAxis(1))), ((driverController.getRawAxis(1))));
				forward = false;
			}
			
			//brake mode
			if(driverController.getRawAxis(3)>0){
				frontLeftDriveMotor.enableBrakeMode(true);
				frontRightDriveMotor.enableBrakeMode(true);
				rearLeftDriveMotor.enableBrakeMode(true);
				rearRightDriveMotor.enableBrakeMode(true);
			}
			else{
				frontLeftDriveMotor.enableBrakeMode(false);
				frontRightDriveMotor.enableBrakeMode(false);
				rearLeftDriveMotor.enableBrakeMode(false);
				rearRightDriveMotor.enableBrakeMode(false);
			}
			
			//Shooter / Lifter
			if (opController.getRawButton(2)){
				opController.setRumble(RumbleType.kLeftRumble, 1);
				shotCounter = 0;
				shooterOn = true;
				shooterLifterMotor.changeControlMode(TalonControlMode.Speed);
				shooterLifterMotor.enable();
				shooterLifterMotor.set(shooterSpeed);
				
			}			
			if (opController.getPOV()==270){
				if(disableAdjust == false)
				{
					shooterSpeed = shooterSpeed-10;
					shooterLifterMotor.set(shooterSpeed);
					disableAdjust = true;
				}
			}
			else if (opController.getPOV()==90){
				if(disableAdjust == false)
				{
					shooterSpeed = shooterSpeed+10;
					shooterLifterMotor.set(shooterSpeed);
					disableAdjust = true;
				}
			}
			else{
				disableAdjust = false;
			}

			if (opController.getRawButton(4) && shooterOn == false){
				shooterLifterMotor.setP(0);
				shooterLifterMotor.setI(0);
				shooterLifterMotor.setD(0);
				shooterLifterMotor.changeControlMode(TalonControlMode.PercentVbus);
				shooterLifterMotor.set(-1);
				shooterLifterMotor.enable();
				
			}
			if (!opController.getRawButton(4) && shooterOn == false){
				shooterLifterMotor.set(0);
			}
			if(opController.getRawButton(1)){
				shooterOn = false;
				shooterLifterMotor.disable();
				opController.setRumble(RumbleType.kLeftRumble, 0);
			}
			
			//Intake Forward and Backwards
			if (opController.getRawButton(5)/*&& up==false*/){
				intakeMotor.set(1);
			}
			else if (opController.getRawAxis(2) > 0/* && up == false*/){
				intakeMotor.set(-1);
			}
			else{
				intakeMotor.set(0);
			}
			
			//Spin Carousel
			if(opController.getRawButton(6)){
				
					if(jam==false){
					carouselMotor.set(0.6);
					if(carouselMotor.getOutputCurrent()>= 14){
						jam=true;
						jamTime = Timer.getFPGATimestamp();
					}
					}
				if (jam==true){
					if(Timer.getFPGATimestamp()-jamTime>= 0.8){
						jam=false;
						jamTime=0;
					}
					carouselMotor.set(-0.4);
					
				}
				
			
			}
		    if(opController.getRawAxis(3) > 0){
				carouselMotor.set(-0.2);
			}
			if(!opController.getRawButton(6) && !opController.getRawButton(8)) {
				carouselMotor.set(0);
			}
			
			//Pneumatic gear lifter up and down
			if(opController.getPOV()==0){
				gearSolenoid.set(DoubleSolenoid.Value.kForward);
				up = true;
				SmartDashboard.putBoolean("Gear Lift UP", true);
			}
			if(opController.getPOV() == 180){
				gearSolenoid.set(DoubleSolenoid.Value.kReverse);
				up = false;
				SmartDashboard.putBoolean("Gear Lift UP", false);
			}
			
			//Lock Unlock Intake 
			if(opController.getRawButton(9)){
				intakeSolenoid.set(DoubleSolenoid.Value.kForward);
			}
			if(opController.getRawButton(10)){
				intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
			}
			if(opController.getRawButton(8)){
				leds.set(true);
			}
			if(opController.getRawButton(7)){
				leds.set(false);
			}
			Timer.delay(0.005); // wait for a motor update time
		}
	}
	

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}
}
