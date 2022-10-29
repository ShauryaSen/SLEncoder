/************************ PROJECT PHIL ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Shooter {
        SmartNumber RING_RPM = new SmartNumber("Shooter/Ring RPM", 3000);

        public interface ShooterPID {

            SmartNumber kP = new SmartNumber("Shooter/Shooter kP", 0);
            SmartNumber kI = new SmartNumber("Shooter/Shooter kI", 0);
            SmartNumber kD = new SmartNumber("Shooter/Shooter kD", 0);

        }

        public interface ShooterFF {

            SmartNumber kS = new SmartNumber("Shooter/ Shooter kS", 0);
            SmartNumber kV = new SmartNumber("Shooter/ Shooter kV", 0);
            SmartNumber kA = new SmartNumber("Shooter/ Shooter kA", 0);

        }

        public interface FeederPID {

            SmartNumber kP = new SmartNumber("Shooter/Feeder kP", 0);
            SmartNumber kI = new SmartNumber("Shooter/Feeder kP", 0);
            SmartNumber kD = new SmartNumber("Shooter/Feeder kP", 0);

        }

        public interface FeederFF {

            SmartNumber kS = new SmartNumber("Shooter/Feeder kS", 0);
            SmartNumber kV = new SmartNumber("Shooter/Feeder kV", 0);
            SmartNumber kA = new SmartNumber("Shooter/Feeder kA", 0);

            SmartNumber FEEDER_RPM_MULTIPLIER = new SmartNumber("Shooter/Feeder RPM Multiplier", 1);

        }
    }
}
