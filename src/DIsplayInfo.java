import java.awt.Color;
import lejos.hardware.Battery;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

class MeasureBrightness{
	private EV3ColorSensor colorSensor;
	public MeasureBrightness(EV3ColorSensor sensor) {
		this.colorSensor = sensor;
	}
	public int measureBrightness() {
		SampleProvider colorProvider = colorSensor.getRedMode();
		float[] sample = new float[colorProvider.sampleSize()];
		colorSensor.fetchSample(sample, 0);
		int brightness = (int)(sample[0] * 100);
		return brightness;
	}
	public void closeColorSensor() {
		colorSensor.close();
	}
}

class VehicleController{
	private static EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
	private static EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.C);
	
	private static int startSpeed = 250;
	private static int targetThreshold;
	
	private static double Kp = 0.1;
	private static double Ki = 0.01;
	private static double Kd = 0.01;
	private static double previousError = 0;
	private static double integral = 0;
	
	public static void setThreshold(int target) {
		targetThreshold = target;
	}
	public static void startVehicle(TextLCD lcd) {
		motorLeft.resetTachoCount();
		motorRight.resetTachoCount();
		motorLeft.setSpeed(startSpeed);
		motorRight.setSpeed(startSpeed);
		motorLeft.forward();
		motorRight.forward();
		
		Thread pidThread = new Thread(() ->{
			while(Button.ENTER.isUp()) {
				pidRegler();
			}
		});
		pidThread.start();
		Button.ENTER.waitForPressAndRelease();
		stopMotors();
	}
	
	private static void pidRegler() {
	    MeasureBrightness measure = new MeasureBrightness(DIsplayInfo.colorSensor);
	    int currentBrightness = measure.measureBrightness();
	    int error = targetThreshold - currentBrightness;

	    integral += error;
	    double derivative = error - previousError;
	    
	    int maxIntegral = 100;
	    integral = Math.max(Math.min(integral, maxIntegral), -maxIntegral);
	    
	    double kpFaktor = Kp * error;
	    double kiFaktor = Ki * integral;
	    double kdFaktor = Kd * derivative;
	    
	    int output = (int)(kpFaktor + kiFaktor + kdFaktor);

	    if (currentBrightness < targetThreshold) { 
	        stopMotors();
	        searchForLine();
	    } else {
	        adjustMotors(output);	
	    }
	    previousError = error;
	}

	private static void searchForLine() {
	    int orgPauseTime = 500;
	    int pauseTime = orgPauseTime;
	    boolean lineFound = false;
	    int tmpPauseTimeCondition = 1;
	    boolean reversePauseTime = true;
	    while(!lineFound) {
	    	motorLeft.setSpeed(50);
	    	motorRight.setSpeed(50);
	    	motorLeft.forward();
	    	motorRight.backward();
	    	
	    	lineFound = waitForLineDetection(pauseTime / 100);
	    	if(lineFound) break;
	    	
	    	if(tmpPauseTimeCondition == 1) {
	    		pauseTime = 1500;
	    		++tmpPauseTimeCondition;
	    	} else if(reversePauseTime && tmpPauseTimeCondition != 1) {
	    		pauseTime = orgPauseTime + 250;
	    		reversePauseTime = false;
	    	} else {
	    		pauseTime += 250;
	    	}
	    	
	    	motorLeft.setSpeed(50);
	    	motorRight.setSpeed(50);
	    	motorLeft.backward();
	    	motorRight.forward();
	    	
	    	lineFound = waitForLineDetection(pauseTime / 100);
	    	if(lineFound) break;
	    	
	    	pauseTime += 250;
	    }
    }
	
	private static boolean waitForLineDetection(int intervals) {
		MeasureBrightness measure = new MeasureBrightness(DIsplayInfo.colorSensor);
		for(int i = 0; i < intervals; ++i) {
			try {
				Thread.sleep(100);
			} catch(InterruptedException e) {
				Thread.currentThread().interrupt();
				return false;
			}
			if(measure.measureBrightness() >= targetThreshold) {
				return true;
			}
		}
		return false;
	}
	
	private static void adjustMotors(int correction) {
		int newSpeedLeft = startSpeed - correction;
		int newSpeedRight = startSpeed + correction;
		
		if(correction > 0) {
			motorLeft.setSpeed(Math.max(0, newSpeedRight));
			motorRight.setSpeed(Math.max(0, newSpeedLeft));
		} else {
			motorLeft.setSpeed(Math.max(0, newSpeedLeft));
			motorRight.setSpeed(Math.max(0, newSpeedRight));
		}
		
		motorLeft.forward();
		motorRight.forward();
	}

	
	public static void stopMotors() {
		motorLeft.stop(true);
		motorRight.stop(true);
	}
	
	public static void closeConnection() {
		motorLeft.close();
		motorRight.close();
	}
}


class Calibrate{	
	public static void calibrate(TextLCD lcd) {
		MeasureBrightness measure = new MeasureBrightness(DIsplayInfo.colorSensor);
		lcd.drawString("Line measurement", 0, 0);
		Button.waitForAnyPress();
		lcd.clear();
		int lineBrightness = measure.measureBrightness();
		lcd.drawString("Linebright.: " + lineBrightness, 0, 0);
		Button.waitForAnyPress();
		lcd.clear();
		
		lcd.drawString("Floor measurement", 0, 0);
		Button.waitForAnyPress();
		lcd.clear();
		int floorBrightness = measure.measureBrightness();
		lcd.drawString("Floorbright.: " + floorBrightness, 0, 0);
		Button.waitForAnyPress();
		lcd.clear();
		
		int targetThreshold = (lineBrightness + floorBrightness) / 2;
		lcd.drawString("TargetThresh. " + targetThreshold, 0, 0);
		VehicleController.setThreshold(targetThreshold);
		Button.waitForAnyPress();
		lcd.clear();
	}
}

class Controller{
	private TextLCD lcd = LocalEV3.get().getTextLCD();	
	public Controller() {
		start();
	}
	private void start() {
		boolean go = true;
        try {
        	do {
                lcd.drawString("Press ENTER", 0, 0);
                Button.waitForAnyPress();
                lcd.clear();

                lcd.drawString("Start calibrating", 0, 0);
                Button.waitForAnyPress();
                lcd.clear();

                Calibrate.calibrate(lcd);

                lcd.drawString("Finished calibr.", 0, 0);
                Button.waitForAnyPress();
                lcd.clear();

                lcd.drawString("ENTER for Start", 0, 0);
                Button.waitForAnyPress();
                VehicleController.startVehicle(lcd);
               

                lcd.drawString("ESC to exit", 0, 0);
                int pressedButton = Button.waitForAnyPress();
                lcd.clear();
                if (pressedButton == Button.ID_ESCAPE) {
                	go = false;
                	throw new RuntimeException("0");
                }
            } while (go);
        } catch (RuntimeException e) {
            System.out.println("Program exited: " + e.getMessage());
        }
    }
}

public class DIsplayInfo {
	public static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
	public static void main(String[] args) {
		Runtime.getRuntime().addShutdownHook(new Thread(() ->{
			TextLCD lcd = LocalEV3.get().getTextLCD();
			lcd.drawString("Cleanup operation...", 0, 0);
			 VehicleController.stopMotors();
			 VehicleController.closeConnection();
			 MeasureBrightness cSensor = new MeasureBrightness(colorSensor);
			 cSensor.closeColorSensor();
		}));
		new Controller();
	}
}
