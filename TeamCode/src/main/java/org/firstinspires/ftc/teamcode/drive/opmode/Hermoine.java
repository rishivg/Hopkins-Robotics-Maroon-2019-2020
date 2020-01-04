package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Adwith")

public class Hermoine extends LinearOpMode {

    CyGoat goat = new CyGoat();
    ToggleMap togMap1 = new ToggleMap();
    MaraudersMap maurMap1 = new MaraudersMap();

    ToggleMap togMap2 = new ToggleMap();
    MaraudersMap maurMap2 = new MaraudersMap();

    boolean left_bumper_toggle = false;
    boolean right_bumper_toggle = false;

    ////////////////////////////////////////////////////////////////////////
    /* V * A * R * I * A * B * E * S *////* V * A * R * I * A * B * E * S */
    ////////////////////////////////////////////////////////////////////////

    /*IMU Variables*/
    double newZero = 0;
    int fullRotationCount = 0;
    double previousAngle = 0;
    int lin_pos = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        goat.init(hardwareMap);
        goat.imu();

        goat.front_left.setDirection(DcMotor.Direction.REVERSE);
        goat.front_right.setDirection(DcMotor.Direction.FORWARD);
        goat.back_left.setDirection(DcMotor.Direction.REVERSE);
        goat.back_right.setDirection(DcMotor.Direction.FORWARD);

        goat.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        goat.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goat.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goat.back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goat.back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        goat.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        goat.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        goat.rotator.setPosition(0.9);
        goat.clawOne.setPosition(0.78);
        goat.clawTwo.setPosition(0);
        goat.pusher.setPosition(0);

        while(!opModeIsActive()){}

        while(opModeIsActive()){
            angleOverflow(); //Keep at the beginning of teleop loop
            drive();
            intake();
            slide();
            pusher();
            latch();
            claw();
            rotator();
            updateKeys();
            telemetry.update();//THIS GOES AT THE END
        }
    }

    // DRIVE CODE //

    public void drive(){

        double protate = 1 * gamepad1.right_stick_x;
        double stick_x = gamepad1.left_stick_x; //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = -gamepad1.left_stick_y; //Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2)
        double magnitudeMultiplier = 0;

        double theta = Math.atan2(stick_y, stick_x) - Math.PI/4; //Arctan2 doesn't have bad range restriction

        double thetaInFirstQuad = Math.abs(Math.atan(stick_y/stick_x)); //square to circle conversion
        if(thetaInFirstQuad > Math.PI/4){
            magnitudeMultiplier = Math.sin(thetaInFirstQuad); //Works because we know y is 1 when theta > Math.pi/4
        }
        else if(thetaInFirstQuad <= Math.PI/4){
            magnitudeMultiplier = Math.cos(thetaInFirstQuad); //Works because we know x is 1 when theta < Math.pi/4
        }

        double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * Math.sqrt(2); //Multiplied by (1-Protate) so it doesn't go over 1 with rotating
        //double Px = magnitude * Math.cos(modifiedTheta - Math.PI/2);
        //double Py = magnitude * Math.sin(modifiedTheta);

        //if (abs.(gamepad1.left_stick_y) = 1) {

        //telemetry.pr
        //}

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Theta", theta);
        //telemetry.addData("Modified Theta", modifiedTheta);
        telemetry.addData("Magnitude",  magnitude);
        telemetry.addData("Multiplier", magnitudeMultiplier);
        telemetry.addData("Front Left", magnitude * Math.cos(theta) + protate);
        telemetry.addData("Back Left", magnitude * Math.sin(theta) + protate);
        telemetry.addData("Back Right", magnitude * Math.cos(theta) - protate);
        telemetry.addData("Front Right", magnitude * Math.sin(theta) - protate);

        double frontLeftPower = magnitude * Math.cos(theta) + protate;
        double frontRightPower = magnitude * Math.sin(theta) - protate;
        double backLeftPower = magnitude * Math.sin(theta) + protate;
        double backRightPower = magnitude * Math.cos(theta) - protate;

        goat.front_left.setPower(frontLeftPower);
        goat.back_left.setPower(backLeftPower);
        goat.back_right.setPower(backRightPower);
        goat.front_right.setPower(frontRightPower);
    }

    public void pusher() {

        if (gamepad2.dpad_left) {

            goat.pusher.setPosition(0);
        }

        if (gamepad2.dpad_right) {

            goat.pusher.setPosition(0.87);
        }
    }

    public void latch() {

        if (gamepad1.dpad_up) {

            goat.latchLeft.setPosition(0);
            goat.latchRight.setPosition(1);
        }

        if (gamepad1.dpad_down) {

            goat.latchLeft.setPosition(0.75);
            goat.latchRight.setPosition(0.25);
        }
    }

    public void slide() {
        if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {

            goat.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (goat.slide.getCurrentPosition() <= 0 && gamepad2.left_trigger > 0) {

                goat.slide.setPower(0);
            } else {
                goat.slide.setPower(gamepad2.right_trigger + -(gamepad2.left_trigger));
            }
        }

        if (gamepad2.dpad_up) {

            goat.slide.setTargetPosition(1270);
            goat.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            goat.slide.setPower(0.8);
            while(Math.abs(goat.slide.getCurrentPosition() - 970) > 60) {
                angleOverflow(); //Keep at the beginning of teleop loop
                drive();
                intake();
                pusher();
                latch();
                claw();
                rotator();
                updateKeys();
                telemetry.update();
            }
            goat.slide.setPower(0);
            goat.rotator.setPosition(0.16);

        }

        if(gamepad2.dpad_down){
            goat.slide.setTargetPosition(1270);
            goat.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            goat.slide.setPower(0.8);
            while(Math.abs(goat.slide.getCurrentPosition() - 970) > 60) {
                angleOverflow(); //Keep at the beginning of teleop loop
                drive();
                intake();
                pusher();
                latch();
                claw();
                rotator();
                updateKeys();
                telemetry.update();
            }
            goat.slide.setPower(0);

            goat.rotator.setPosition(0.9);
            double startRotate = getRuntime();
            double finishedRotate = getRuntime();
            while(finishedRotate - startRotate < 1){
                finishedRotate = getRuntime();
            }

            goat.slide.setTargetPosition(-40);
            goat.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            goat.slide.setPower(0.4);
            while(Math.abs(goat.slide.getCurrentPosition()- 0) > 10){
                angleOverflow(); //Keep at the beginning of teleop loop
                drive();
                intake();
                pusher();
                latch();
                claw();
                rotator();
                updateKeys();
                telemetry.update();
            }
            goat.slide.setPower(0);

        }

        if (gamepad2.x) {

            goat.slide.setTargetPosition(530);
            goat.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            goat.slide.setPower(0.7);
            while(Math.abs(goat.slide.getCurrentPosition() - 530) > 10){
            }
            goat.slide.setPower(0);



        }

        if (gamepad2.y) {

            goat.slide.setTargetPosition(-40);
            goat.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            goat.slide.setPower(0.4);
            while(Math.abs(goat.slide.getCurrentPosition()- 0) > 10){
                angleOverflow(); //Keep at the beginning of teleop loop
                drive();
                intake();
                pusher();
                latch();
                claw();
                rotator();
                updateKeys();
                telemetry.update();
            }
            goat.slide.setPower(0);
            // goat.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }


        telemetry.addData("Slide pos", goat.slide.getCurrentPosition());

        telemetry.addData("Slide", (gamepad2.right_trigger + -(gamepad2.left_trigger)));

        // low: 520
    }

    public void intake() {

        if (gamepad1.left_bumper) {

            goat.intake1.setPower(0.4);
            goat.intake2.setPower(-0.4);
        }

        if (gamepad1.right_bumper) {

            goat.intake1.setPower(-0.4);
            goat.intake2.setPower(0.4);
        }

        if (gamepad1.y) {

            goat.intake1.setPower(0);
            goat.intake2.setPower(0);
        }

        telemetry.addData("Intake", (gamepad1.right_trigger + -(gamepad1.left_trigger)));
    }

    public void claw() {

        if(gamepad2.a) {

            goat.clawOne.setPosition(0.78);
            goat.clawTwo.setPosition(0);
        }

        if(gamepad2.b) {

            goat.clawOne.setPosition(0);
            goat.clawTwo.setPosition(0.6);
        }
    }

    public void rotator() {

        if(gamepad2.left_bumper) {

            goat.rotator.setPosition(0.16);
        }

        if(gamepad2.right_bumper) {

            goat.rotator.setPosition(0.9);
        }
    }


    public void angleOverflow(){ //Increase fullRotationCount when angle goes above 2*PI or below 0
        double heading = getHeading() - fullRotationCount*(2*Math.PI);
        //Warning: Will break if the robot does a 180 in less thank 1 tick, but that probably won't happen
        if(heading < Math.PI/4 && previousAngle > 3*Math.PI/4){
            fullRotationCount++;
        }
        if(heading > 3*Math.PI/4 && previousAngle < Math.PI/4){
            fullRotationCount--;
        }
        previousAngle = heading;
    }
    public double getHeading(){ //Includes angle subtraction, angle to radian conversion, and 180>-180 to regular system conversion
        Orientation angles = goat.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        heading = (Math.PI/180)*heading;
        if(heading < 0){
            heading = (2*Math.PI) + heading;
        }
        heading = heading - newZero;

        heading += fullRotationCount*(2*Math.PI);
        return heading;
    }

    // TOGGLES ////////// USE MAP //

    public void updateKeys(){
        if(gamepad1.a && cdCheck(maurMap1.a, 1000)){
            togMap1.a = toggle(togMap1.a);
            maurMap1.a = runtime.milliseconds();
        }
        if(gamepad1.b && cdCheck(maurMap1.b, 500)){
            togMap1.b = toggle(togMap1.b);
            maurMap1.b = runtime.milliseconds();
        }
        if(gamepad2.b && cdCheck(maurMap2.b, 500)){
            togMap2.b = toggle(togMap2.b);
            maurMap2.b = runtime.milliseconds();
        }
        if(gamepad1.right_stick_x > 0 && cdCheck(maurMap1.right_stick_right, 700)){
            togMap1.right_stick_right = toggle(togMap1.right_stick_right);
            maurMap1.right_stick_right = runtime.milliseconds();
        }
    }

    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }
    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }
}