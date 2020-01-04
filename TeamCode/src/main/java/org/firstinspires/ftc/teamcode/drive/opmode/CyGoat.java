package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CyGoat {


    //DRIVE//
    public DcMotor front_left   = null;
    public DcMotor front_right  = null;
    public DcMotor back_left    = null;
    public DcMotor back_right   = null;

    public DcMotor intake1 = null;
    public DcMotor intake2 = null;

    public DcMotor slide = null;

    public Servo clawOne = null;
    public Servo clawTwo = null;

    public Servo rotator = null;

    public Servo latchLeft = null;
    public Servo latchRight = null;

    public Servo pusher = null;

    BNO055IMU imu;

    HardwareMap hwMap = null;

    /* Constructor */
    public CyGoat(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        /* RETRIEVING STUFF FROM PHONES */

        //DRIVE//
        front_left   = hwMap.dcMotor.get("front_left");
        front_right  = hwMap.dcMotor.get("front_right");
        back_left    = hwMap.dcMotor.get("back_left");
        back_right   = hwMap.dcMotor.get("back_right");

        intake1 = hwMap.dcMotor.get("intake_left");
        intake2 = hwMap.dcMotor.get("intake_right");

        slide = hwMap.dcMotor.get("slide");

        clawOne = hwMap.servo.get("clawOne");
        clawTwo = hwMap.servo.get("clawTwo");

        rotator = hwMap.servo.get("rotator");

        latchLeft = hwMap.servo.get("latchLeft");
        latchRight = hwMap.servo.get("latchRight");

        pusher = hwMap.servo.get("pusher");


        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        slide.setDirection(DcMotor.Direction.FORWARD);

        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.FORWARD);
        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //IMU//
        imu = hwMap.get(BNO055IMU.class, "imu");

        // Add sum more stuff//
    }
    public void imu(){
        /* IMU STUFF */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu.initialize(parameters);
    }
}