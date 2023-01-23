package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.SystemClock;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="teleOP!!!!!!!!")
public class teleOP extends OpMode {


    public static DcMotor tLeftDT  = null;
    public static DcMotor bLeftDT  = null;
    public static DcMotor tRightDT = null;
    public static DcMotor bRightDT = null;
    //public static DcMotor Lift     = null;
    //public static DcMotor Claw     = null;
    public static Servo scan = null;
    public static DigitalChannel red = null;
    public static DigitalChannel green = null;
    public static RevTouchSensor touch = null;
    public static Rev2mDistanceSensor range = null;
    public static ColorSensor color = null;
    public static IMU imu = null;


    public double speedMode = 0.9;
    public boolean xIsHeld = false;
    public boolean bIsHeld = false;
    public boolean dpadLeftIsHeld = false;
    public boolean dpadRightIsHeld = false;
    public double spd = 0.4;

    public long lastTime = 0;
    public boolean lockerA = false;
    public boolean lockerB = false;



    @Override
    public void init() {
        telemetry.clearAll();
        telemetry.addData("Status", "TeleOP Initialization In Progress");
        telemetry.update();

        //Hardware map
        tLeftDT   = hardwareMap.get(DcMotor.class, "FrontL");
        bLeftDT   = hardwareMap.get(DcMotor.class, "BackL");
        tRightDT  = hardwareMap.get(DcMotor.class, "FrontR");
        bRightDT  = hardwareMap.get(DcMotor.class, "BackR");
        //Lift      = hardwareMap.get(DcMotor.class, "lift"    );
        //Claw      = hardwareMap.get(DcMotor.class,   "claw"    );
        scan = hardwareMap.get(Servo.class, "scanner");
        red = hardwareMap.get(DigitalChannel.class, "red");
        green = hardwareMap.get(DigitalChannel.class, "green");
        touch = hardwareMap.get(RevTouchSensor.class, "touch");
        range = hardwareMap.get(Rev2mDistanceSensor.class, "range");
        color = hardwareMap.get(ColorSensor.class, "color");
        imu = hardwareMap.get(IMU.class, "imu");

        scan.setPosition(0.5);

        color.enableLed(false);

        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
        red.setState(true);
        green.setState(true);


        tRightDT.setDirection(DcMotor.Direction.REVERSE);
        tLeftDT.setDirection(DcMotor.Direction.REVERSE);
        bRightDT.setDirection(DcMotor.Direction.FORWARD);
        bLeftDT.setDirection(DcMotor.Direction.FORWARD);
//        Lift.setDirection(DcMotor.Direction.FORWARD);
//        Claw.setDirection(DcMotor.Direction.FORWARD);



        tRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        tRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        tRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        bLeftDT.setPower(0);
        tLeftDT.setPower(0);
        bRightDT.setPower(0);
        tRightDT.setPower(0);
//        Lift.setPower(0);
//        Claw.setPower(0);




        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }


    @Override
    public void loop() {

        //Slow Mode Code for a and b keys
//        if (gamepad1.a) {
//            speedMode = .6;
//        }
//        if (gamepad1.b) {
//            speedMode = 1;
//        }
        //Slow Mode Code for a and b keys

        //Slow Mode Code for bumpers
//        if (gamepad1.right_bumper && speedMode > .2) {
//            speedMode -= .05;
//        } else if (gamepad1.right_trigger >= .5 && speedMode < 2) {
//            speedMode += .05;
//        }
        //Slow Mode Code for bumpers
        if (gamepad1.a) {
            if (!lockerA) {
                lockerA = true;
                green.setState(!green.getState());
            }
        }
        else
            lockerA = false;

        if (gamepad1.b) {
            if (!lockerB) {
                lockerB = true;
                red.setState(!red.getState());
            }
        }
        else
            lockerB = false;

        if (gamepad1.x && !gamepad1.y) {
            scan.setPosition(0.4);
        }

        if (gamepad1.y && !gamepad1.x) {
            scan.setPosition(0.6);
        }

        if (!gamepad1.y && ! gamepad1.x)
            scan.setPosition(0.5);

        double stopBuffer = 0; //Not currently Implemented



        //Drive Train Code
        double forward = speedMode * Math.pow(gamepad1.left_stick_y, 3);
        double right = -speedMode * Math.pow(gamepad1.left_stick_x, 3);
        double turn = -speedMode * Math.pow(gamepad1.right_stick_x,3);

        double leftFrontPower = forward + right + turn;
        double leftBackPower = forward - right + turn;
        double rightFrontPower = forward - right - turn;
        double rightBackPower = forward + right - turn;
        double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        boolean needToScale = false;
        for (double power : powers){
            if(Math.abs(power) > 1){
                needToScale = true;
                break;
            }
        }
        if (needToScale){
            double greatest = 0;
            for (double power : powers){
                if (Math.abs(power) > greatest){
                    greatest = Math.abs(power);
                }
            }
            leftFrontPower /= greatest;
            leftBackPower /= greatest;
            rightFrontPower /= greatest;
            rightBackPower /= greatest;
        }

        boolean stop = true;
        for (double power : powers){
            if (Math.abs(power) > stopBuffer){
                stop = false;
                break;
            }
        }
        if (stop){
            leftFrontPower = 0;
            leftBackPower = 0;
            rightFrontPower = 0;
            rightBackPower = 0;
        }


        if (!gamepad1.y) {
            tLeftDT.setPower(tLeftDT.getPower()*0.80+leftFrontPower*.20);
            bLeftDT.setPower(bLeftDT.getPower()*0.80+leftBackPower*.20);
            tRightDT.setPower(tRightDT.getPower()*0.80+rightFrontPower*.20);
            bRightDT.setPower(bRightDT.getPower()*0.80+rightBackPower*.20);
        } else {
            driveAngle(135, 0.3);
        }
        //Drive Train Code


        //lift code
//        Lift.setPower(4*gamepad2.left_stick_y);



        //wormhole in
        if(gamepad2.y){
            spd += 0.1;
        }
        if(gamepad2.a){
            spd -= 0.1;
        }

//        if (gamepad2.b) {
//            Claw.setPower(spd);
//
//        }else if (gamepad2.x) {
//            Claw.setPower(-spd);
//
//        }else {
//            Claw.setPower(0);
//
//        }
        // 537.7 encoder counts per revolution of the output shaft of the motor
        // 7.46 encoder counts is estimated backlash of final gear
        // Estimated "good enough" position is +/-4 encoder counts, can this be used with isBusy?
        telemetry.addLine("Front left encoder counts: " + tLeftDT.getCurrentPosition());
        telemetry.addLine("Back left encoder counts: " + bLeftDT.getCurrentPosition());
        telemetry.addLine("Front right encoder counts: " + tRightDT.getCurrentPosition());
        telemetry.addLine("Back right encoder counts: " + bRightDT.getCurrentPosition());
        telemetry.addLine("Distance: " + range.getDistance(DistanceUnit.CM));
        telemetry.addLine("Angle: " + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        long currentTime = SystemClock.currentThreadTimeMillis();
        telemetry.addLine("Loop Time (ms):" + (currentTime - lastTime));
        lastTime = currentTime;
//        telemetry.addLine("Lift encoder counts: " + Lift.getCurrentPosition());
//        telemetry.addLine("Claw encoder counts: " + Claw.getCurrentPosition());

    }

    void driveAngle(double degrees, double power) {
        // Add 45 to degrees to get sin and cos values aligned with wheel powers, convert to radians
        double radians = ((degrees+45) * Math.PI ) / 180;
        double sin = Math.sin(radians);
        double cos = Math.cos(radians);

        // Normalize & apply power factor
        double max = Math.max(Math.abs(sin), Math.abs(cos));
        sin = sin / max * power;
        cos = cos / max * power;

        tLeftDT.setPower(-sin);
        bLeftDT.setPower(-cos);
        tRightDT.setPower(-cos);
        bRightDT.setPower(-sin);
    }

    void scan() {
        //SystemClock.currentThreadTimeMillis()
        scan.setPosition(0);
    }
}