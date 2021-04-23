package org.firstinspires.ftc.teamcode;
/* Created Atul Errabolu and Kush on 7/25/2019 */
/* modified by Sebastian Ochoa on 1/3/21 */
//Modified by Aman Modi on 1/4/21
//restructured by Seb on 1/5/21
// this is a test for pull requests.
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//test lol Test
//for jon in jon on jon
//test 2
@TeleOp
public class TeleOP extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, arm, shooter, intake, transfer;
    private boolean direction, togglePrecision;
    private ColorSensor color_sensor;
    private Rev2mDistanceSensor Distance;
    private double factor;
    boolean currentB = false;
    boolean driveOne = false;
    boolean previousB = false;
    boolean currentRB = false;
    boolean previousRB = false;
    boolean clawState = false;
    boolean flicktime = false;
    double armPos = 0;
    private Servo claw, flicker, holder, cheese; //claw, flicker, holder
    boolean reverse;
    OpenCvInternalCamera phoneCam;
    TeleOP.Autoaim pipeline;
    private PID forwardPID = new PID(.022, 0, 0.0033);
    private final int WHEEL_RADIUS = 4;
    private final double GEAR_RATIO = (double)5/6;
    private final double TICKS_PER_REVOLUTION = 537.6;
    private final double TPI = TICKS_PER_REVOLUTION/(2*Math.PI*GEAR_RATIO*WHEEL_RADIUS);
    private PID strafePID = new PID(.015, 0, 0.003);

    int reverseFactor;
    private BNO055IMU imu;
    private ElapsedTime runtime;
    private double servo;
    double shooterPower = .77;
    int balls = 2;
    boolean shotMode = false;
    boolean clawReady = false;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime clawmove = new ElapsedTime();
    ElapsedTime flick = new ElapsedTime();
    ElapsedTime ringCheck = new ElapsedTime();
    boolean servoMoving = false;
    boolean armmove = false;
    final static double dropWobbleTime = 1000;
    Orientation angles;
    boolean CheeseWas = false;
    boolean CheeseIs = false;
    boolean CheeseState = false;
    boolean CheeseUp = true;
    boolean shootinTime = false;
    int rings = 0;
    double ringsWas = 0;
    double ringsIs = 1;
    boolean yell = false;

//rando comment


    double initialAngle ;
    @Override
    public void init() {
        //Maps all the variables to its respective hardware
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFrontDrive");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftRearDrive");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFrontDrive");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightRearDrive");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        transfer = (DcMotorEx) hardwareMap.dcMotor.get("transfer");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        claw = hardwareMap.servo.get("claw");
        flicker = hardwareMap.servo.get("flicker");
        holder = hardwareMap.servo.get("holder");
        cheese = hardwareMap.servo.get("cheese");
        color_sensor = hardwareMap.colorSensor.get("colorSensor");
        Distance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "hopper");

        //Initialize all the hardware to use Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initializing all new motors (shooter, arm, intake, transfer)
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Setting the motors' power to 70

        //Initialize the motors to begin stationary
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Left Motors are in reverse and Right Motors are forward so the robot can move forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        runtime = new ElapsedTime();
        reverse = false;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new TeleOP.Autoaim();
        phoneCam.setPipeline(pipeline);
        phoneCam.openCameraDeviceAsync(() -> {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        });

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initialAngle=angles.firstAngle;
        cheese.setPosition(.46);

    }


    @Override
    public void loop() {

        //Increasing the power gradually
        //int power = (DcMotorSimple) arm.getPower();
        //toggles precision mode if the right stick button is pressed
        CheeseIs = gamepad2.a;
        if(CheeseIs && !CheeseWas){
            CheeseState = true;
        }
        ringsIs = Distance.getDistance(DistanceUnit.MM);
        //sets the factor multiplied to the power of the motors
        factor = togglePrecision ? .3 : 1; //the power is 1/5th of its normal value while in precision mode
        if(!leftFront.isBusy()&&!leftBack.isBusy()&&!rightFront.isBusy()&&!rightBack.isBusy()) {
            // Do not mess with this, if it works, it works
            double x = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double stickAngle = Math.atan2(direction ? -gamepad1.left_stick_y : gamepad1.left_stick_y, direction ? gamepad1.left_stick_x : -gamepad1.left_stick_x); // desired robot angle from the angle of stick
            double powerAngle = stickAngle - (Math.PI / 4); // conversion for correct power values
            double rightX = gamepad1.right_stick_x; // right stick x axis controls turning
            final double leftFrontPower = Range.clip(x * Math.cos(powerAngle) + rightX, -.80, .80);
            final double leftRearPower = Range.clip(x * Math.sin(powerAngle) + rightX, -.80, .80);
            final double rightFrontPower = Range.clip(x * Math.sin(powerAngle) - rightX, -.80, .80);
            final double rightRearPower = Range.clip(x * Math.cos(powerAngle) - rightX, -.80, .80);


            //Set the position of arm to counter clockwise/clockwise


            //neutral is .5, right trigger .5 to 1, left trigger is 0 to .5 What???


            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setPower(leftFrontPower * factor);
            leftBack.setPower(leftRearPower * factor);
            rightFront.setPower(rightFrontPower * factor);
            rightBack.setPower(rightRearPower * factor);
        }
        //Incrementing the power by 0.0 EVERY TIME you call this function
        //Incrementing the power by 0.0 EVERY TIME you call this function
        // for jon in jon on jon

        //Updating the power of the motors
     /* arm.setPower(power);
     shooter.setPower(power);
     intake.setPower(power);
     transfer.setPower(power); */


        //autoaim();
        //Reset the intake and transfer encoders
        precisionMode(); //check for precision mode
        singlePlayer(); //check to see if player one takes over
        armTravel(); // move arm
        powerShot(); // toggles speed mode for flywheel
        revShoot(); // controls flywheel
        toggleIntake(); // controls intake, on off backwards
        flickRing(); // toggles flicker
        toggleHolder(); // toggles intake clip
        armTravel2(); // second player arm function
        snapBot();
        cheeseStick();
        rocketFart();
        countRings();
        //speak();



        telemetry.addData("Rings:", rings);
        telemetry.addData("Power shot mode:", getShotMode());
        telemetry.addData("One driver: ", getDrive());
        telemetry.addData("RB",gamepad1.right_bumper);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("angle",angles.firstAngle);
        telemetry.update();



        CheeseWas = CheeseIs;
        ringsWas = rings;

        //}
    }



    public void countRings(){
        if(ringsWas != rings){
            ringCheck.reset();
            yell = true;
        }
        //hmm
        if(ringCheck.milliseconds() > 400) {
            if (ringsIs > 80) {
                rings = 0;
                if(yell) {
                    telemetry.speak("none");
                    yell = false;
                }
            } else if (ringsIs > 50) {
                rings = 1;
                if(yell) {
                    telemetry.speak("one");
                    yell = false;
                }
            } else if (ringsIs > 28) {
                rings = 2;
                if(yell) {
                    telemetry.speak("two");
                    yell = false;
                }
            } else if (ringsIs > 15) {
                rings = 3;
                if(yell) {
                    telemetry.speak("three");
                    yell = false;
                }
            } else {
                if(yell) {
                    telemetry.speak("fuck");
                    yell = false;
                }
            }

        }

    }
    public void autoaim(){
        snapBot();
        boolean move = true;
        color_sensor.enableLed(true);  // Turn the LED on
        while(color_sensor.alpha() < 20){
            moveByWheelEncoders(0, 1, .4, "straight", move);
            move = false;
            telemetry.addData("Alpha", color_sensor.alpha());
            telemetry.update();
        }
        color_sensor.enableLed(false); // Turn the LED off

    }


    public void rocketFart()
    {



        if(gamepad1.b) {
            shootinTime = true;
            flick.reset();
        }
        if(shootinTime){
            shooter.setPower(-.8);
            if (flick.milliseconds() <= 1200) {
                if (flick.milliseconds() > 0 && flick.milliseconds() < 170)
                    flicker.setPosition(.35);
                if (flick.milliseconds() > 200 && flick.milliseconds() < 370) {
                    shooter.setPower(-.78);
                    flicker.setPosition(.64);
                }
                if (flick.milliseconds() > 400 && flick.milliseconds() < 570)
                    flicker.setPosition(.35);
                if (flick.milliseconds() > 600 && flick.milliseconds() < 770) {
                    shooter.setPower(-.73);
                    flicker.setPosition(.64);
                }
                if (flick.milliseconds() > 800 && flick.milliseconds() < 970)
                    flicker.setPosition(.35);
            }
            if(flick.milliseconds() > 1200) {
                flicker.setPosition(.64);
                shooter.setPower(0);
                shootinTime = false;
            }
        }
    }

    public void moveByWheelEncoders(double targetHeading, double inches, double power, String movementType, boolean move)  {
        if(move)
            resetMotors();

        double currentPosition = leftBack.getCurrentPosition();

        int ticks = (int)(inches * TPI);

        while (motorsBusy(ticks, currentPosition)) {
            correction(-power, targetHeading, movementType, false, 1);
        }

        // halt();
    }
    public void strafeByWheelEncoders(double targetHeading, double inches, double power, String movementType)  {
        resetMotors();

        double currentPosition = leftBack.getCurrentPosition();

        int ticks = (int)(inches * TPI * 1.2);

        while (motorsBusy(ticks, currentPosition)) {
            correction(-power, targetHeading, movementType, false, 1);
        }

        halt();
    }

    public boolean motorsBusy(int ticks, double startingPosition) {
        return Math.abs(leftBack.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightBack.getCurrentPosition() - startingPosition) < ticks && Math.abs(leftFront.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightFront.getCurrentPosition() - startingPosition) < ticks;
    }

    public double currentAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void correction(double power, double targetHeading, String movementType, boolean inverted, double max) {
        //sets target and current angles
        double target = targetHeading;
        double current = currentAngle();

        //if the spline motion is backwards, the target must be flipped 180 degrees in order to match with spline.getAngle()
        if (inverted && movementType.contains("spline")) {
            target = (targetHeading > 0) ? (targetHeading - 180) : (targetHeading + 180);
        }

        //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
        double error = getError(current, target);

        //PD correction for both regular and spline motion
        if (movementType.contains("straight") || movementType.contains("spline")) {
            double correction = forwardPID.getCorrection(error, runtime);

            double leftPower = Range.clip(power - correction, -max, max);
            double rightPower = Range.clip(power + correction, -max, max);
            double nearing = 0;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);
//            telemetry.addData("left expected power", leftPower);
//            telemetry.addData("right expected power", rightPower);
//            telemetry.addData("actual left power", leftFront.getPower());
//            telemetry.addData("actual right power", rightFront.getPower());
        }

        //pd correction for strafe motion. Right and left are opposites

        else if (movementType.contains("strafe")) {
            double correction = strafePID.getCorrection(error, runtime);

            if (movementType.contains("left")) {
                leftFront.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(power + correction, -1.0, 1.0));
                leftBack.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightBack.setPower(Range.clip(-power + correction, -1.0, 1.0));
            } else if (movementType.contains("right")) {
                leftFront.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(-power + correction, -1.0, 1.0));
                leftBack.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightBack.setPower(Range.clip(power + correction, -1.0, 1.0));
            }
        }

    }

    public void halt () {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public double getError(double current, double target) {
        double error;

        double error1 = /*current - target*/ target - current;
        double error2;
        if (current < 0)
            error2 = /*(current + 360) - (target);*/ target - (current + 360);
        else
            error2 = /*(current - 360) - (target);*/ target - (current - 360);
        if (Math.abs(error1) <= Math.abs(error2))
            error = error1;
        else
            error = error2;

        return error;
    }


    public void resetMotors(){
        DcMotorEx[] motors = new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack};

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

    public void revShoot() { // controls the flywheel WORKS
        if (gamepad1.right_trigger > .499999) {
            shooter.setPower(-shooterPower);
        } else {
            shooter.setPower(0);
        }
    }
    public void cheeseStick(){
        if(CheeseState && CheeseUp){
            CheeseState = false;
            cheese.setPosition(.82);
            CheeseUp = false;
        }
        if(CheeseState && !CheeseUp){
            CheeseState = false;
            cheese.setPosition(.46);
            CheeseUp = true;
        }
    }

    public void powerShot() { // lowers flywheel speed
        if(driveOne) {
            if (gamepad1.dpad_left && !shotMode) {
                shooterPower = .71;
                shotMode = true;
            } else if (gamepad1.dpad_right && shotMode) {
                shooterPower = .77;
                shotMode = false;
            }
        }
        if(!driveOne) {
            if (gamepad1.y && !shotMode) {
                shooterPower = .71;
                shotMode = true;
            } else if (gamepad1.x && shotMode) {
                shooterPower = .77;
                shotMode = false;
            }
        }
    }

    public void toggleIntake() { // controls intake and transfer
        if(driveOne) {
            if (gamepad1.left_trigger > .499999) {
                intake.setPower(-1);
                transfer.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(1);
                transfer.setPower(-1);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }
        } else if(!driveOne){
            if (gamepad2.left_trigger > .499999) {
                intake.setPower(-1);
                transfer.setPower(1);
            } else if (gamepad2.left_bumper) {
                intake.setPower(1);
                transfer.setPower(-1);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }
        }
    }
    public void singlePlayer() {
        if(gamepad1.start){
            driveOne = true;
        }
        if(gamepad1.back){
            driveOne = false;
        }
    }

    public void flickRing() { // controls flicker
 /* if (gamepad1.a && !servoMoving) {
 timer.reset();
 flicker.setPosition(1);
 servoMoving = true;
 }
 if (timer.milliseconds() >= 850 && servoMoving) {
 timer.reset();
 flicker.setPosition(0);
 servoMoving = false;
 }
*/      if(!shootinTime) {
            if (gamepad1.a) {
                flicker.setPosition(.35);
            } else {
                flicker.setPosition(.64);
            }
        }// use this code if the above code refuses to work.

     /*   if (gamepad1.a){
           flick.reset();
           flicktime = true;
        }
        if(flicktime){
            if(flick.milliseconds() > 0 && flick.milliseconds() < 250){
                flicker.setPosition(.35);
            }
            if(flick.milliseconds() > 300 && flick.milliseconds() < 550){
                flicker.setPosition(.64);
            }
            if(flick.milliseconds() > 600 && flick.milliseconds() < 850){
                flicker.setPosition(.35);
            }
            if(flick.milliseconds() > 900 && flick.milliseconds() < 1150){
                flicker.setPosition(.64);
            }
            if(flick.milliseconds() > 1200 && flick.milliseconds() < 1450){
                flicker.setPosition(.35);
            }
            if(flick.milliseconds() > 1500){
                flicker.setPosition(.64);
                flicktime = false;
            }
        }
*/
    }

    public void precisionMode() { // controls precision mode
        if (gamepad2.left_stick_button || gamepad1.left_stick_button) {
            togglePrecision = true;
        } else if (gamepad2.right_stick_button || gamepad1.right_stick_button) {
            togglePrecision = false;
        }
    }

    public void snapBot() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if(gamepad1.left_bumper){
            double turnAmount=-angles.firstAngle;//right is negative

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            int turnFactor = (int) (turnAmount/Math.abs(turnAmount));
            double initialPower;
            double minSpeed;
            if(Math.abs(angles.firstAngle)<=85&&Math.abs(angles.firstAngle)>45){
                initialPower=.5;
                minSpeed=.2;
            }
            else if(Math.abs(angles.firstAngle)<=45){
                initialPower=.4;
                minSpeed=.1;
            }
            else{
                initialPower=1;
                minSpeed=.4;
            }
            leftFront.setPower(-turnFactor*initialPower);
            rightFront.setPower(turnFactor*initialPower);
            leftBack.setPower(-turnFactor*initialPower);
            rightBack.setPower(turnFactor*initialPower);

            telemetry.addData("turnAmount",turnAmount);
            telemetry.addData("currentPower",leftFront.getPower());
            telemetry.addData("angle difference", angles.firstAngle);
            telemetry.addData("currentAngle",angles.firstAngle);
            telemetry.update();

            while(Math.abs(angles.firstAngle)>5){ //make if statement
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                leftFront.setPower(-Range.clip((Math.abs(angles.firstAngle)/Math.abs(turnAmount)),minSpeed,initialPower)*turnFactor);//current angle-amount we want to turn =
                rightFront.setPower(Range.clip((Math.abs(angles.firstAngle)/Math.abs(turnAmount)),minSpeed,initialPower)*turnFactor);
                leftBack.setPower(-Range.clip((Math.abs(angles.firstAngle)/Math.abs(turnAmount)),minSpeed,initialPower)*turnFactor);
                rightBack.setPower(Range.clip((Math.abs(angles.firstAngle)/Math.abs(turnAmount)),minSpeed,initialPower)*turnFactor);
                telemetry.addData("turnAmount",turnAmount);
                telemetry.addData("currentPower",leftFront.getPower());
                telemetry.addData("angle difference", angles.firstAngle);
                telemetry.addData("currentAngle",angles.firstAngle);
                telemetry.update();
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void toggleHolder() {// not really useful, just to make it possible to toggle the holder if needed.
        if(driveOne) {
            if (gamepad1.dpad_up) {
                holder.setPosition(1);
            }
            if (gamepad1.dpad_down) {
                holder.setPosition(0);
            }
        }
        if (gamepad2.dpad_up) {
            holder.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            holder.setPosition(0);
        }
    }



    public void armTravel() { // controls arm WORKS
        if(driveOne) {

            if (gamepad1.y) {
                if (armPos == 0) {
                    arm.setTargetPosition(-2268);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setPower(-.9);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armPos = 1;
                    clawState = true;
                    clawmove.reset();
                }
                if (armPos == 0.5) {
                    arm.setTargetPosition(-756);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setPower(-.9);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armPos = 1;
                    clawState = true;
                    clawmove.reset();
                }
            }

            if (gamepad1.x) {
                if (armPos == 1) {
                    clawmove.reset();
                    claw.setPosition(1);
                    clawReady = true;
                    clawState = false;

                }

            }
            if (gamepad1.b) {
                if (armPos == 0) {
                    arm.setTargetPosition(-1512);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setPower(-.9);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armPos = 0.5;
                    clawState = true;
                    clawmove.reset();
                }
            }


            if (clawState && clawmove.milliseconds() > 1000) {
                claw.setPosition(0);
            }
            if (clawReady) {
                if (clawmove.milliseconds() > 800) {
                    arm.setTargetPosition(2268);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setPower(.9);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armPos = 0;
                    armmove = false;
                    clawReady = false;
                }
            }
        }
    }

    public void armTravel2(){
        if(!driveOne){
            if(gamepad2.y){
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-.6);
            }
            else if(gamepad2.x){
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(.6);
            } else {

                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(arm.getCurrentPosition());
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.2);
            }
            if (gamepad2.right_trigger > .499999) {
                claw.setPosition(0);
            } else {
                claw.setPosition(1);
            }

        }
    }
    public boolean checkB()
    {

        if (currentB) previousB = true;
        else previousB = false;
        currentB = gamepad1.b;


        if(currentB && !previousB){
            return true;
        }
        return false;
    }
    public void speak() {
        if(gamepad2.b){
            telemetry.speak("FOR JON IN JON ON JON");
        }
    }

//    public boolean checkRB()
//    {
//
//        if (currentRB) previousRB = true;
//        else previousRB = false;
//        currentRB = gamepad1.right_bumper;
//
//
//        if(currentRB && !previousRB){
//            return true;
//        }
//        return false;
//    }
//    public void powerShots(){ //needs testing
//        if(gamepad1.f){
//            //use bettermovebot if possible. WIP
//            moveBot(1,2,2,1,44,.6,false); //will be to better movement updated later
//            shooter.setPower(-shooterPower);
//            flicker.setPosition(0);
//            flicker.setPosition(0.7);
//            shooter.setPower(0);
//            moveBot(1,2,2,1,8,.6,false); //will be updated to better movement later
//            shooter.setPower(-shooterPower);
//            flicker.setPosition(0);
//            flicker.setPosition(0.7);
//            shooter.setPower(0);
//            moveBot(1,2,2,1,8,.6,false); //will be updated to better movement later
//            shooter.setPower(-shooterPower);
//            flicker.setPosition(0);
//            flicker.setPosition(0.7);
//            shooter.setPower(0);
//        }
//    }

    //    public void moveBot(int leftT, int rightT, int leftB, int rightB, int distance, double power, boolean withIntake){
//        //moveBot(1, 1, 2, 2, -24, .60, true); //Forwward
//        //turnBot(2, 2, 1, 1, -24, .60); //Backward
//        //turnBot(2, 1, 2, 1, 30, .60); //Strafe right
//        //turnBot(1, 2, 1, 2, 0, .6) /Strafe left
//        if (leftT == 1) {
//            leftFront.setDirection(DcMotor.Direction.FORWARD);
//        } else if(leftT == 2){
//            leftFront.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (leftB == 1) {
//            leftBack.setDirection(DcMotor.Direction.FORWARD);
//        } else if(leftB == 2){
//            leftBack.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (rightT == 2) {
//            rightFront.setDirection(DcMotor.Direction.FORWARD);
//        } else if(rightT == 1){
//            rightFront.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        if (rightB == 2) {
//            rightBack.setDirection(DcMotor.Direction.FORWARD);
//        } else if(rightT == 1){
//            rightBack.setDirection(DcMotor.Direction.REVERSE);
//        }
//
//        /*if(withIntake) {
//            while (shooterTime.milliseconds() <= 5000) {
//                intake.setPower(.5);
//                transfer.setPower(1);
//                heartbeat();
//            }
//        }*/
//
//        //Moves the robot
//        int travel = (int)(distance * TPI);
//        for (DcMotorEx motor : motors) {
//            motor.setTargetPosition(travel);
//            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motor.setPower(power);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        //intake.setPower(0);
//        // transfer.setPower(0);
//    }
// public double currentAngle() {
// return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
// }
    public boolean getShotMode()
    {
        return shotMode;
    }

    public boolean getDrive()
    {
        return driveOne;
    }


    public static class Autoaim extends OpenCvPipeline {



        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(115, 115);

        static final int REGION_WIDTH = 5;
        static final int REGION_HEIGHT = 25;


        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);

        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;


        void inputToCb(Mat input){
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 0);
        }

        @Override
        public void init(Mat firstFrame){
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);



            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }



    }


}
