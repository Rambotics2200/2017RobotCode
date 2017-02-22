package org.usfirst.frc.team2200.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot2{
	private final Object imgLock = new Object();
	double count1, count2;
	double hue[], sat[], lum[];
	DigitalOutput LED = new DigitalOutput(PinsClass.leds);
	UsbCamera camera;
	double leftX, rightX, centerX;
	private RobotDrive myRobot;
	
	GripPipeline gp;
	
	public Robot2(RobotDrive d) {
		myRobot = d;
		
		LED.set(true);
		
		gp = new GripPipeline();
		
		gp.setHSL(20.0, 75.0, 44.0, 131.0, 160.0, 255.0);
		count1 = 0.0;
		count2 = 0.0;
		hue = new double[2];
		sat = new double[2];
		lum = new double[2];
		
		SmartDashboard.putNumber("Hue min", 30.0);
		SmartDashboard.putNumber("Hue max", 75.0);
		
		SmartDashboard.putNumber("Sat min", 130.0);
		SmartDashboard.putNumber("Sat max", 250.0);

		SmartDashboard.putNumber("Lum min", 160.0);
		SmartDashboard.putNumber("Lum max", 255.0);

        
	}

	


	
	public void followCamera() {

		
		double startTime = Timer.getFPGATimestamp();
		double endTime = startTime+1;
		while(endTime > Timer.getFPGATimestamp()){
			Timer.delay(0.005);
		}
		camera.setWhiteBalanceHoldCurrent();
		camera.setExposureHoldCurrent();
		
		
		
        double count[] = new double[2];
        
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Processed", 640, 480);
        
        Mat source = new Mat();
        Mat output = new Mat();
        
        
        
		
		boolean keepGoing = true;
		
		while (keepGoing) {
			double leftX, rightX, centerX;
			//myRobot.arcadeDrive(stick); // drive with arcade style (use right
										// stick)
			
			
			//get image and process
			cvSink.grabFrame(source);
            gp.process1(source);
            //gp.process(source);
            outputStream.putFrame(gp.hslThresholdOutput());
            gp.process2();
            
            count[0] = gp.filterContoursOutput().size();
            count[1] = gp.findContoursOutput().size();
            //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
            if (!gp.filterContoursOutput().isEmpty() && count[0] >= 2) {
                Rect r = Imgproc.boundingRect(gp.filterContoursOutput().get(0));
                Rect r1 = Imgproc.boundingRect(gp.filterContoursOutput().get(1));
                
	            synchronized (imgLock) {
	            	leftX = r.x + (r.width / 2);
	            	rightX = r1.x + (r1.width / 2);
	            	centerX = (leftX + rightX) / 2;
	            }
                
            }
            
            synchronized (imgLock) {
                count1 = count[0];
                count2 = count[1];
            	gp.setHSL(hue[0], hue[1], sat[0], sat[1], lum[0], lum[1]);
            }
			
            //finish get image and proceses

        	synchronized (imgLock) {
    			hue[0] = SmartDashboard.getNumber("Hue min",0.0);
    			hue[1] = SmartDashboard.getNumber("Hue max",0.0);
    			sat[0] = SmartDashboard.getNumber("Sat min",0.0);
    			sat[1] = SmartDashboard.getNumber("Sat max",0.0);
    			lum[0] = SmartDashboard.getNumber("Lum min",0.0);
    			lum[1] = SmartDashboard.getNumber("Lum max",0.0);
        		
        		count[0] = count1;
        		count[1] = count2;
	            
	            leftX = this.leftX;
	            rightX = this.rightX;
	            centerX = this.centerX;
        	}

			SmartDashboard.putNumber("count 1",count[0]);
			SmartDashboard.putNumber("count 2",count[1]);
			SmartDashboard.putNumber("left", leftX);
			SmartDashboard.putNumber("right", rightX);
			SmartDashboard.putNumber("center", centerX);
			

			if (centerX > 330.0) {
				myRobot.tankDrive(-0.5, 0.5);
			}
			else if (centerX < 310.0) {
				myRobot.tankDrive(0.5, -0.5);
			}
			else {
				myRobot.tankDrive(0.0, 0.0);
			}
			
			
			Timer.delay(0.005); // wait for a motor update time
		}
		LED.set(false);
		
	}

	
}
