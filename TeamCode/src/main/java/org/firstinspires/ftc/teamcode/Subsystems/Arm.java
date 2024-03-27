package org.firstinspires.ftc.teamcode.Subsystems;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Claw;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Tele_Red;

public class Arm {
    private Shoulder shoulder;
    private Telescope telescope;
    private Claw claw;
    private Lift lift;
    private Wrist wrist;
    private AHRS navx;
    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern mainColor;
    private RevBlinkinLedDriver.BlinkinPattern violet = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
    private int state = 0;
    private boolean autoPickup = false;
    private ElapsedTime colorTimer = new ElapsedTime();
    private boolean isRedHere;

    public Arm(Shoulder s, Telescope t, Claw c, Lift l, Wrist w, AHRS n, RevBlinkinLedDriver b, boolean fromAuto, boolean isRed) {
        this.shoulder = s;
        this.telescope = t;
        this.claw = c;
        this.lift = l;
        this.wrist = w;
        this.navx = n;
        this.blinkin = b;
        this.isRedHere = isRed;
        if (isRed) {
            this.mainColor = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
        } else {
            this.mainColor = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        }
        this.blinkin.setPattern(this.mainColor);
    }

    public void goTo_Initialization(){
        this.wrist.setTarget(RC_Wrist.stowPos, RC_Wrist.stowTime);
        this.claw.closeBottom();
        this.claw.closeTop();
        this.shoulder.setPosition(200);
    }

    public void goTo_Pickup(boolean holding){
        if (this.state != 1) {
            if (!holding) {
                this.claw.openBottom();
                this.claw.openTop();
            }
            this.wrist.setTarget(RC_Wrist.pickupPos, RC_Wrist.pickupTime);
            this.state = -1;

        }
    }

    public void goTo_Stow(boolean holding) throws InterruptedException {
        if (this.state == -1) {
            this.claw.closeBottom();
            this.claw.closeTop();
            Thread.sleep(RC_Claw.pickupPause);
            this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        } else if (this.state == 1) {
            this.claw.partialBottom();
            Thread.sleep(RC_Claw.dropBottomPause);
            this.claw.partialTop();
            Thread.sleep(RC_Claw.dropTopPause);
            this.shoulder.setPosition(RC_Shoulder.stowPos);
            this.telescope.setPosition(RC_Telescope.stowPos);
            this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        } else if (this.state == 0) {
            this.shoulder.setPosition(RC_Shoulder.stowPos);
            this.telescope.setPosition(RC_Telescope.stowPos);
            this.wrist.setTarget(RC_Wrist.stowPos,RC_Wrist.stowTime);
        }
        this.state = 0;
    }

    public void goTo_dropOff(boolean holding){
        if (this.state == 0){
            if (TelemetryData.whatHeadingDo < -4.4 || TelemetryData.whatHeadingDo > 1.5){
                //we are facing the board
                this.shoulder.setPosition(RC_Shoulder.dropOffPosRev);
                this.telescope.setPosition(RC_Telescope.dropOffHighRev);
                this.wrist.setTarget(RC_Wrist.dropOffPosRev, RC_Wrist.dropOffTime);
            } else {
                //we are facing normally
                this.shoulder.setPosition(RC_Shoulder.dropOffPos);
                this.telescope.setPosition(RC_Telescope.dropOffHigh);
                this.wrist.setTarget(RC_Wrist.dropOffPos, RC_Wrist.dropOffTime);
            }
            this.state = 1;
        }
    }

    public void prepForLift(double input) {
        //input = input*.9;
        if (TelemetryData.liftStage == -3) {
            //move shoulder up to clear the cord
            this.shoulder.move(-input,true);
            this.telescope.move(0,true);
            if (TelemetryData.shoulder_position > 200) {
                TelemetryData.liftStage = -2;
            }
        } else if (TelemetryData.liftStage == -2) {
            //move telescope in to clear the cord
            this.shoulder.move(0,true);
            this.telescope.move(input, true);
            if (TelemetryData.telescope_position < 5) {
                TelemetryData.liftStage = -1;
            }
        } else if (TelemetryData.liftStage == -1) {
            //move shoulder down
            this.shoulder.move(input, true);
            this.telescope.move(input*.3, true);
            if (TelemetryData.shoulder_position < 5) {
                TelemetryData.liftStage = 0;
            }
        } else if (TelemetryData.liftStage == 0) {
            //move telescope out
            this.shoulder.move(input*.3, true);
            this.telescope.move(-input, true);
            if (TelemetryData.telescope_position > 300) {
                TelemetryData.liftStage = 1;
            }
        } else if (TelemetryData.liftStage == 1) {
            //move shoulder up
            this.telescope.move(0,true);
            this.shoulder.move(-input*.7, true);
            if (TelemetryData.shoulder_position > 1200) {
                TelemetryData.liftStage = 2;
                TelemetryData.shoulder_target = 1400;
            }
        } else if (TelemetryData.liftStage == 2) {
            //move telescope out
            this.shoulder.update(1);
            this.telescope.move(-input*.7, true);
            if (TelemetryData.telescope_position > 1800) {
                TelemetryData.liftStage = 3;
                TelemetryData.telescope_target = 2000;
                TelemetryData.shoulder_target = 1400;
                this.telescope.move(0,true);
                this.shoulder.move(0,true);
            }
        } else {
            this.shoulder.update(1);
            this.telescope.update(1);
        }






            /*
            //move down to engage the cord
            TelemetryData.shoulder_target = 0;
            //TelemetryData.telescope_target = 0;
            this.telescope.move(input,true);
            this.shoulder.move(-input,true);
            if (Math.abs(TelemetryData.shoulder_target - TelemetryData.shoulder_position) < 15) {
                TelemetryData.liftStage = -10;
            }
        } else if (TelemetryData.liftStage == -1) {
            //push telescope out to engage cable
            if (TelemetryData.shoulder_position > 0) {
                this.shoulder.move(-.3,true);
            }
            if (TelemetryData.shoulder_position < 10) {
                TelemetryData.telescope_target = 300;
            }
            if (TelemetryData.telescope_position > 250) {
                TelemetryData.liftStage = 0;
            }
        }

             */

                /*
                this.shoulder.update();
                this.telescope.manualMove(-input);
                this.telescope.getCurrent();
                if (TelemetryData.telescope_current>600){
                    double modifier = 1-(800-TelemetryData.telescope_current)/200.;
                    this.lift.moveManual(-modifier);
                } else {
                    this.lift.moveManual(0);
                }
            } else {
                TelemetryData.liftStage = 0;
                this.telescope.manualMove(0);
                this.shoulder.update();
                this.lift.moveManual(0);
            }
        } else if (TelemetryData.liftStage == 0) {
            //move shoulder into position to lift
            this.telescope.manualMove(0);
            if (TelemetryData.telescope_position < 300) {
                this.shoulder.update();
                this.telescope.manualMove(-input);
            } else if (TelemetryData.telescope_position > 400) {
                this.shoulder.update();
                this.telescope.manualMove(input * .2);
            }
            if (TelemetryData.shoulder_position < 1400) {
                double speed = this.shoulder.moveManual(-input);
                this.shoulder.getCurrent();
                //this.telescope.manualMove(speed / 3.33);
                this.telescope.manualMove(0);
                if (TelemetryData.shoulder_current > 2250) {
                    double modifier = 1 - (2550 - TelemetryData.shoulder_current) / 300. + .1;
                    this.lift.moveManual(-modifier);
                } else {
                    this.lift.moveManual(0);
                }
            } else {
                TelemetryData.liftStage = 1;
                this.telescope.manualMove(0);
                this.shoulder.moveManual(0);
                this.lift.moveManual(0);
            }
        } else if (TelemetryData.liftStage == 1) {
            //move telescope up to above bar
            this.shoulder.updatePosition();
            this.telescope.updatePosition();
            this.shoulder.moveManual(0);
            if (TelemetryData.telescope_position < RC_Telescope.prepForLift) {
                if (TelemetryData.shoulder_position < 1400) {
                    this.shoulder.moveManual(-input);
                } else if (TelemetryData.shoulder_position > 1500){
                    this.shoulder.moveManual(input*.2);
                } else {
                    this.shoulder.moveManual(0);
                }


                this.telescope.manualMove(-input);
                this.telescope.getCurrent();
                if (TelemetryData.telescope_current > 600) {
                    double modifier = 1 - (800 - TelemetryData.telescope_current) / 200.;
                    this.lift.moveManual(-modifier);
                }
            } else {
                TelemetryData.liftStage = 2;
                this.telescope.manualMove(0);
                this.shoulder.moveManual(0);
                this.lift.moveManual(0);
            }
        }else if (TelemetryData.liftStage == 2) {
            this.telescope.setPosition(RC_Telescope.prepForLift);
            this.shoulder.setPosition(1500);
            //updateEverything();
            //this.lift.moveManual(0);
        }

             */

    }

    public void lifting(double input){
        if (input != 0) {
            //we are trying to lift
            input = input *.5;
            double pitch = getPitch();

            if (pitch > 1) {
                //backend low
            } else if (pitch < -1) {
                //backend high
            } else {

            }
        }


        /*
        if (Math.abs(input)>0 && pitch<-.01) {
            double modifier = input + (pitch/-.07)*.5;
            this.lift.move(-modifier);
            if (TelemetryData.telescope_position < 350){
                this.telescope.move(0,true);
            }
        } else if (Math.abs(input)>0 && pitch > .01) {
            double modifier = input - (pitch/.07)*.5;
            this.lift.moveManual(modifier);
            if (TelemetryData.telescope_position < 350){
                this.telescope.manualMove(0);
            }
        } else {
            this.lift.moveManual(input);
            if (TelemetryData.telescope_position < 350){
                this.telescope.manualMove(0);
            } else {
                this.telescope.manualMove(-input);
            }
        }

         */
    }

    public double getPitch(){
        return this.navx.getRoll();
    }
}