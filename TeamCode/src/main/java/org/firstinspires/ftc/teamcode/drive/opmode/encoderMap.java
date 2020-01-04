package org.firstinspires.ftc.teamcode.drive.opmode;

public class encoderMap{
    double range = 1;
    double voltageToAngle = 2*Math.PI/3.28;

    double startingVoltage = 0;
    double previousVoltage = 0;
    double rotationCounter = 0;
    double direction = 0;

    double previousAngle = 0;
    double angle = 0;
    double deltaAngle = 0;

    public encoderMap(){}
    public void init(double voltage){
        startingVoltage = voltage;
        previousVoltage = voltage;
    }
    public void update(double voltage){
        if(previousVoltage < range && voltage > 3.28 - range){
            rotationCounter--;
        }
        else if(previousVoltage > 3.28 - range && voltage < range){
            rotationCounter++;
        }
        else if(Math.abs(voltage-previousVoltage) > range){
            rotationCounter += direction;
        }
        if(voltage < 3 && voltage > 0.3 && previousVoltage < 3 && previousVoltage > 0.3){
            if(voltage - previousVoltage > 0.1){
                direction = 1;
            }
            if(voltage - previousVoltage < -0.1){
                direction = -1;
            }
        }
        previousVoltage = voltage;
        angle = voltageToAngle*(rotationCounter*3.28+voltage-startingVoltage);
        deltaAngle = angle-previousAngle;
        previousAngle = angle;
    }
}