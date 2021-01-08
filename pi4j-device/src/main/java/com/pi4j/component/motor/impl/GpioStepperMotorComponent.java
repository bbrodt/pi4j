package com.pi4j.component.motor.impl;

/*
 * #%L
 * **********************************************************************
 * ORGANIZATION  :  Pi4J
 * PROJECT       :  Pi4J :: Device Abstractions
 * FILENAME      :  GpioStepperMotorComponent.java
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
import com.pi4j.component.motor.StepperMotorBase;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinState;

public class GpioStepperMotorComponent extends StepperMotorBase {

	public enum InterfaceType {
		// This is a Stepper Motor Driver board like the TB6600. This board requires 3 pins:
		// Pin 1 = Enable/Disable (disabled state means the motor is free to turn "by hand")
		// Pin 2 = Direction (forward/reverse)
		// Pin 3 = Step Pulse
		DRIVER,
		// this is a 4-wire full-step motor
		FULL4WIRE,
		// this is a 4-wire half-step motor
		HALF4Wire
	};
	
    // internal class members
    private GpioPinDigitalOutput pins[];
    private PinState onState;
    private PinState offState;
    private MotorState currentState = MotorState.STOP;
    private GpioStepperMotorControl controlThread = new GpioStepperMotorControl();
    private int sequenceIndex = 0;
    private long stepCount = 0;
    private long stepIndex = 0;
    private InterfaceType type;
	// state of Enable pin for DRIVER Interface Types
	private boolean enabled = true; 

    /**
     * using this constructor requires that the consumer
     *  define the STEP ON and STEP OFF pin states
     *
     * @param pins GPIO digital output pins for each controller in the stepper motor
     * @param onState pin state to set when MOTOR STEP is ON
     * @param offState pin state to set when MOTOR STEP is OFF
     */
    public GpioStepperMotorComponent(GpioPinDigitalOutput pins[], PinState onState, PinState offState, InterfaceType type) {
        this.pins = pins;
        this.onState = onState;
        this.offState = offState;
        this.type = type;
    }

    /**
     * default constructor; using this constructor assumes that:
     *  (1) a pin state of HIGH is MOTOR STEP ON
     *  (2) a pin state of LOW  is MOTOR STEP OFF
     *  (3) if number of pins==3, assume a Stepper Motor Driver
     *      if number of pins==4, assume a 4-wire stepper motor
     *
     * @param pins GPIO digital output pins for each controller in the stepper motor
     */
    public GpioStepperMotorComponent(GpioPinDigitalOutput pins[]) {
        this(pins, PinState.HIGH, PinState.LOW, pins.length==3 ? InterfaceType.DRIVER : InterfaceType.FULL4WIRE);
    }

    /**
     * Return the current motor state
     *
     * @return MotorState
     */
    @Override
    public MotorState getState() {
        return currentState;
    }

    /**
     * change the current stepper motor state
     *
     * @param state new motor state to apply
     */
    @Override
    public void setState(MotorState state) {

    	if (currentState != state) {
	        switch(state) {
	            case STOP: {
	                // set internal tracking state
	                currentState = MotorState.STOP;
	
	                // turn all GPIO pins to OFF state
	                if (type==InterfaceType.DRIVER) {
	                	for (int i=1; i<pins.length; ++i) {
	                		// leave the Enable pin HIGH!
	                		// This forces the Stepper Motor Driver to
	                		// hold the motor at the current position.
	                        pins[i].setState(offState);
	                	}
	                }
	                else {
	                    for(GpioPinDigitalOutput pin : pins)
	                        pin.setState(offState);
	                }
	                break;
	            }
	            case FORWARD: {
	                // set internal tracking state
	                currentState = MotorState.FORWARD;
	
	                // start control thread if not already running
	                if(!controlThread.isAlive()) {
	                    controlThread = new GpioStepperMotorControl();
	                    controlThread.start();
	                }
	
	                break;
	            }
	            case REVERSE: {
	                // set internal tracking state
	                currentState = MotorState.REVERSE;
	
	                // start control thread if not already running
	                if(!controlThread.isAlive()) {
	                    controlThread = new GpioStepperMotorControl();
	                    controlThread.start();
	                }
	
	                break;
	            }
	            default: {
	                throw new UnsupportedOperationException("Cannot set motor state: " + state.toString());
	            }
	        }
            notifyStateChangeListeners();
    	}
    }

    private class GpioStepperMotorControl extends Thread {
        public void run() {

        	// enable DRIVER
        	enable(true);
        	
            // continuous loop until stopped
            while(currentState != MotorState.STOP) {
                // control direction
                if(currentState == MotorState.FORWARD)
                    doStep(true);
                else if(currentState == MotorState.REVERSE)
                    doStep(false);
            }

            setState(MotorState.STOP);
        }
    }

    @Override
    public void stepBlocking(long steps)
    {
        // validate parameters
        if (steps == 0) {
            setState(MotorState.STOP);
            return;
        }
        sequenceIndex = 0;

        // perform step in positive or negative direction from current position
        boolean forward = true;
        if (steps < 0){
        	steps = -steps;
        	forward = false;
        }
        for(long index = 0; index < steps; index++)
            doStep(forward);
    }
    
    @Override
    public void step(long steps)
    {
        // validate parameters
        if (steps == 0) {
            setState(MotorState.STOP);
            return;
        }

        // perform step in positive or negative direction from current position
        boolean forward = true;
        if (steps < 0){
        	steps = -steps;
        	forward = false;
        }
        sequenceIndex = 0;
        stepIndex = 0;
        stepCount = steps;

        setState(forward ? MotorState.FORWARD : MotorState.REVERSE);
    }

    /**
     * this method performs the calculations and work to control the GPIO pins
     * to move the stepper motor forward or reverse
     * @param forward
     */
    private void doStep(boolean forward) {

        // increment or decrement sequence
        if(forward)
            sequenceIndex++;
        else
            sequenceIndex--;

        if (stepCount!=0) {
        	// do a specific number of steps
        	if (stepIndex >= stepCount) {
        		// all done
        		setState(MotorState.STOP);
        		return;
        	}
        	++stepIndex;
        }
        
        if (type==InterfaceType.DRIVER) {
        	// set Stepper Motor Driver direction and pulse the step pin
            pins[1].setState(forward ? onState : offState);
            if((sequenceIndex & 1) == 1)
                pins[2].setState(onState);
            else
                pins[2].setState(offState);
        }
        else {
            // check sequence bounds; rollover if needed
            if(sequenceIndex >= stepSequence.length)
                sequenceIndex = 0;
            else if(sequenceIndex < 0)
                sequenceIndex = (stepSequence.length - 1);

            // start cycling GPIO pins to move the motor forward or reverse
	        for(int pinIndex = 0; pinIndex < pins.length; pinIndex++) {
	            // apply step sequence
	            double nib = Math.pow(2, pinIndex);
	            if((stepSequence[sequenceIndex] & (int)nib) > 0)
	                pins[pinIndex].setState(onState);
	            else
	                pins[pinIndex].setState(offState);
	        }
        }
     
        long start = System.nanoTime();
        while (System.nanoTime() - start <= stepIntervalMilliseconds*1000000 + stepIntervalNanoseconds) ;
   
        /*
        try {
            Thread.sleep(stepIntervalMilliseconds, stepIntervalNanoseconds);
        }
        catch (Exception e) {}
        */
    }

	@Override
	public void enable(boolean flag) {
		if (type==InterfaceType.DRIVER) {
			enabled  = flag;
			pins[0].setState(enabled ? onState : offState);
		}
	}

	@Override
	public boolean isEnabled() {
		return enabled;
	}
}
