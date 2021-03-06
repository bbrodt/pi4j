/*
 * #%L
 * **********************************************************************
 * ORGANIZATION  :  Pi4J
 * PROJECT       :  Pi4J :: Java Examples
 * FILENAME      :  StepperMotorGpioExample.java
 *
 * This file is part of the Pi4J project. More information about
 * this project can be found here:  http://www.pi4j.com/
 * **********************************************************************
 * %%
 * Copyright (C) 2012 - 2016 Pi4J
 * %%
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 *
 * You should have received a copy of the GNU General Lesser Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/lgpl-3.0.html>.
 * #L%
 */

import com.pi4j.component.motor.MotorState;
import com.pi4j.component.motor.MotorStateChangeListener;
import com.pi4j.component.motor.impl.GpioStepperMotorComponent;
import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;

/**
 * This example code demonstrates how to control a stepper motor
 * using the GPIO pins on the Raspberry Pi.
 *
 * @author Robert Savage
 */
public class StepperMotorGpioExample {

	private static GpioController gpio;
	private static boolean stateChanged = false;
   
    private static MotorStateChangeListener listener = new MotorStateChangeListener() {

		@Override
		public void motorStateChange(MotorState state) {
			System.out.println("Motor State Changed: "+state.name());
	    	if (state == MotorState.STOP)
	    		stateChanged = true;    		
		}
    };
	
    public static void main(String[] args) throws InterruptedException {
        System.out.println("<--Pi4J--> GPIO Stepper Motor Example ... started.");

        // create gpio controller
        gpio = GpioFactory.getInstance();
        
        System.out.println("DRIVER or FULL4WIRE stepper motor (D/F)?");
        String type = System.console().readLine();
        if (type.contains("d") || type.contains("D"))
        	runDriverExample();
        else if (type.contains("f") || type.contains("F"))
        	runFull4WireExample();
        else
        	System.out.println("incorrect input!");

        // stop all GPIO activity/threads by shutting down the GPIO controller
        // (this method will forcefully shutdown all GPIO monitoring threads and scheduled tasks)
        gpio.shutdown();

        System.out.println("Exiting StepperMotorGpioExample");
    }

    private static void waitForStop(GpioStepperMotorComponent motor) throws InterruptedException {
        while (motor.getState() != MotorState.STOP) {
        	Thread.sleep(1000);
        	System.out.print(".");
        }
        System.out.println();
    }
    
    private static void runDriverExample() throws InterruptedException {
        // provision gpio pins #00 to #02 as Driver Enable, Direction and Pulse
    	// output pins and ensure in LOW state
        final GpioPinDigitalOutput[] pins = {
                gpio.provisionDigitalOutputPin(RaspiPin.GPIO_27, PinState.LOW), // Enable
                gpio.provisionDigitalOutputPin(RaspiPin.GPIO_28, PinState.LOW), // Direction
                gpio.provisionDigitalOutputPin(RaspiPin.GPIO_29, PinState.LOW)  // Pulse step
        };

        // this will ensure that the motor is stopped when the program terminates
        gpio.setShutdownOptions(true, PinState.LOW, pins);

        // create motor component
        GpioStepperMotorComponent motor = new GpioStepperMotorComponent(pins);
        motor.addStateChangeListener(listener);

        // define stepper parameters before attempting to control motor
        motor.setStepInterval(0, 100000);
        //motor.setStepInterval(1);

        // There are 200 steps per revolution for the NEMA 17 motor
        int s = 3200*2;
        motor.setStepsPerRevolution(s);

        //s = 2000;
        for (int r=0; r<2; ++r) {
	        // test motor control : STEPPING FORWARD
	        System.out.println("   Motor FORWARD for "+s+" steps.");
	        motor.step(s);
	        waitForStop(motor);
	        System.out.println("   Motor STOPPED for 1 seconds.");
	        Thread.sleep(1000);
	
	        // test motor control : STEPPING REVERSE
	        System.out.println("   Motor REVERSE for "+s+" steps.");
	        motor.step(-s);
	        waitForStop(motor);
	        System.out.println("   Motor STOPPED for 1 seconds.");
	        Thread.sleep(1000);
        }

        // test motor control : ROTATE FORWARD
        System.out.println("   Motor FORWARD for 10 revolutions.");
        motor.rotate(10);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : ROTATE REVERSE
        System.out.println("   Motor REVERSE for 10 revolutions.");
        motor.rotate(-10);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : TIMED FORWARD
        System.out.println("   Motor FORWARD for 5 seconds.");
        motor.forward(5000);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : TIMED REVERSE
        System.out.println("   Motor REVERSE for 5 seconds.");
        motor.reverse(5000);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : ROTATE FORWARD with different timing and sequence
        System.out.println("   Motor FORWARD with slower speed and higher torque for 10 revolutions.");
        motor.setStepInterval(1);
        motor.rotate(10);
        motor.rotate(-1);
        System.out.println("   Motor STOPPED.");

        // final stop to ensure no motor activity
        motor.stop();
        motor.enable(false);
    }
        
    private static void runFull4WireExample() throws InterruptedException {
        // provision gpio pins #00 to #03 as output pins and ensure in LOW state
        final GpioPinDigitalOutput[] pins = {
                gpio.provisionDigitalOutputPin(RaspiPin.GPIO_21, PinState.LOW),
                gpio.provisionDigitalOutputPin(RaspiPin.GPIO_22, PinState.LOW),
                gpio.provisionDigitalOutputPin(RaspiPin.GPIO_23, PinState.LOW),
                gpio.provisionDigitalOutputPin(RaspiPin.GPIO_24, PinState.LOW)
        };

        // this will ensure that the motor is stopped when the program terminates
        gpio.setShutdownOptions(true, PinState.LOW, pins);

        // create motor component
        GpioStepperMotorComponent motor = new GpioStepperMotorComponent(pins);
        motor.addStateChangeListener(listener);

        // @see http://www.lirtex.com/robotics/stepper-motor-controller-circuit/
        //      for additional details on stepping techniques

        // create byte array to demonstrate a single-step sequencing
        // (This is the most basic method, turning on a single electromagnet every time.
        //  This sequence requires the least amount of energy and generates the smoothest movement.)
        byte[] single_step_sequence = new byte[4];
        single_step_sequence[0] = (byte) 0b0001;
        single_step_sequence[1] = (byte) 0b0010;
        single_step_sequence[2] = (byte) 0b0100;
        single_step_sequence[3] = (byte) 0b1000;

        // create byte array to demonstrate a double-step sequencing
        // (In this method two coils are turned on simultaneously.  This method does not generate
        //  a smooth movement as the previous method, and it requires double the current, but as
        //  return it generates double the torque.)
        byte[] double_step_sequence = new byte[4];
        double_step_sequence[0] = (byte) 0b0011;
        double_step_sequence[1] = (byte) 0b0110;
        double_step_sequence[2] = (byte) 0b1100;
        double_step_sequence[3] = (byte) 0b1001;

        // create byte array to demonstrate a half-step sequencing
        // (In this method two coils are turned on simultaneously.  This method does not generate
        //  a smooth movement as the previous method, and it requires double the current, but as
        //  return it generates double the torque.)
        byte[] half_step_sequence = new byte[8];
        half_step_sequence[0] = (byte) 0b0001;
        half_step_sequence[1] = (byte) 0b0011;
        half_step_sequence[2] = (byte) 0b0010;
        half_step_sequence[3] = (byte) 0b0110;
        half_step_sequence[4] = (byte) 0b0100;
        half_step_sequence[5] = (byte) 0b1100;
        half_step_sequence[6] = (byte) 0b1000;
        half_step_sequence[7] = (byte) 0b1001;

        // define stepper parameters before attempting to control motor
        // anything lower than 2 ms does not work for my sample motor using single step sequence
        motor.setStepInterval(2);
        motor.setStepSequence(single_step_sequence);

        // There are 32 steps per revolution on my sample motor, and inside is a ~1/64 reduction gear set.
        // Gear reduction is actually: (32/9)/(22/11)x(26/9)x(31/10)=63.683950617
        // This means is that there are really 32*63.683950617 steps per revolution =  2037.88641975 ~ 2038 steps!
        motor.setStepsPerRevolution(2000);

        // test motor control : STEPPING FORWARD
        System.out.println("   Motor FORWARD for 2038 steps.");
        motor.step(2038);
        waitForStop(motor);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : STEPPING REVERSE
        System.out.println("   Motor REVERSE for 2038 steps.");
        motor.step(-2038);
        waitForStop(motor);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : ROTATE FORWARD
        System.out.println("   Motor FORWARD for 2 revolutions.");
        motor.rotate(2);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : ROTATE REVERSE
        System.out.println("   Motor REVERSE for 2 revolutions.");
        motor.rotate(-2);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : TIMED FORWARD
        System.out.println("   Motor FORWARD for 5 seconds.");
        motor.forward(5000);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : TIMED REVERSE
        System.out.println("   Motor REVERSE for 5 seconds.");
        motor.reverse(5000);
        System.out.println("   Motor STOPPED for 2 seconds.");
        Thread.sleep(2000);

        // test motor control : ROTATE FORWARD with different timing and sequence
        System.out.println("   Motor FORWARD with slower speed and higher torque for 1 revolution.");
        motor.setStepSequence(double_step_sequence);
        motor.setStepInterval(10);
        motor.rotate(1);
        System.out.println("   Motor STOPPED.");

        // final stop to ensure no motor activity
        motor.stop();
    }
}
