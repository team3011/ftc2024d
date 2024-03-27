package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class Telescope {
    private DcMotorEx motor;
    private TouchSensor touch;
    private PIDController controller;
    private boolean resetting = false;
    private int resetStage = 0;
    private boolean resetTriggered = false;
    private int lastTarget = 0;
    private boolean manualEngaged = false;
    private boolean justTouched = false;

    /**
     * Class constructor
     * @param m motor obj
     * @param t touch sensor obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public Telescope(DcMotorEx m, TouchSensor t, boolean fromAuto) {
        this.motor = m;
        this.touch = t;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.controller = new PIDController(RC_Telescope.kP, RC_Telescope.kI, RC_Telescope.kD);
        this.motor.setPower(0);

        if (fromAuto) {
            if (!this.touch.isPressed()) {
                this.resetStage = 1;
                this.justTouched = true;
            } else {
                TelemetryData.shoulder_position = 0;
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.resetStage = 0;
            }
        }
    }

    public void reset(){
        TelemetryData.telescope_position = 0;
        TelemetryData.telescope_target = 0;
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * only method that moves the telescope
     * @param input positive numbers move the telescope out
     * @param fromJoystick did this command to move come from a joystick
     */
    public void move(double input, boolean fromJoystick) {
        TelemetryData.telescope_position = this.motor.getCurrentPosition();
        TelemetryData.telescope_current = this.motor.getCurrent(CurrentUnit.MILLIAMPS);
        if (fromJoystick && input != 0) {
            TelemetryData.telescope_target = TelemetryData.telescope_position;
        }
        int telescopeMin = (int) (TelemetryData.shoulder_position * .315);
        if (input > 0) {
            if (TelemetryData.telescope_position > RC_Telescope.maxOut) {
                input = 0;
            }
            if (TelemetryData.shoulder_target > 1500 && TelemetryData.shoulder_position < 1500){
                input = 0;
            }
        } else if (input < 0) {
            if (this.touch.isPressed()) {
                input = 0;
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        } else {
            if (TelemetryData.telescope_position < telescopeMin - 10) {
                //we need to move the telescope out
                input = .1 + calcFeedForward();
            } else if (TelemetryData.telescope_position > telescopeMin + 10 &&
                    TelemetryData.telescope_position > TelemetryData.telescope_target) {
                //we need to move the telescope int
                input = -.3 + calcFeedForward();
            } else {
                input = calcFeedForward();
            }
        }
        this.motor.setPower(input);
    }

    public void setPosition(int t){
        TelemetryData.telescope_target = t;
    }

    /**
     * standard update function that will move the shoulder if not at the desired location
     */
    public void update(double lim){
        move(calcPower(lim),false);
    }

    private double calcPower(double lim) {
        double power = 0;
        this.controller.setPID(RC_Telescope.kP, RC_Telescope.kI, RC_Telescope.kD);
        double pid = this.controller.calculate(TelemetryData.telescope_position, TelemetryData.telescope_target);
        pid = pid/10;
        TelemetryData.telescope_pid = limiter(pid, lim);
        double ff = calcFeedForward();
        power = limiter(pid + ff,lim);
        return power;
    }

    /**
     * this will limit the input to a range of -limiter to limiter
     * @param input the value to be limited
     * @param limiter the max value the input can be
     * @return the limited input
     */
    private double limiter(double input, double limiter){
        if (input > limiter) {
            input = limiter;
        } else if (input < -limiter) {
            input = -limiter;
        }
        return input;
    }

    /**
     * calculate the feedforward value by taking the cosine of the target angle relative to the ground
     * This works because cos() represents the ratio between the adjacent side of a right triangle and
     * the hypotenuse of the triangle. Whenever the arm is extended straight out (0 degrees),
     * the value of the cosine function is at its maximum (1). This is the point where the system
     * demands the most torque to hold its weight. Whenever we want the arm to be oriented straight
     * up or down (90 degrees), the arm does not require any torque to hold its weight. This matches
     * our feedforward controller, as cos(90 degrees) is 0.
     * By using a nonlinear feedforward controller, we can improve the reliability of our control
     * system and improve the performance of our system.
     *
     * @return the calculated feedforward value
     */
    private double calcFeedForward(){
        double temp = TelemetryData.shoulder_position / RC_Shoulder.ticksFor90 + RC_Shoulder.startAngle;
        return Math.sin(Math.toRadians(temp)) * RC_Telescope.kF;
    }

    public void getCurrent(){
        TelemetryData.telescope_current = this.motor.getCurrent(CurrentUnit.MILLIAMPS);
    }
}