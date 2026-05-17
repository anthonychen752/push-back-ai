/*----------------------------------------------------------------------------*/
/*                                                                            */
/*   VEX AI Push Back - Main Program                                         */
/*   SEARCH → COLLECT → SCORE state machine                                  */
/*   Odometry + PID + Pure Pursuit                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "robot-config.h"
#include "navigation.h"
#include <cmath>

namespace bex = vex;
using namespace bex;
brain::lcd &BrainScreen = Brain.Screen;

// ═══════════════════════════════════════════════════════════════════════════
// VEXCODE DEVICE CONFIG — mirrors robot-config.cpp for reference
// The actual device definitions are in robot-config.cpp (linked at build time)
// ═══════════════════════════════════════════════════════════════════════════

// Note: motors, sensors, Brain, Competition are declared in robot-config.h
// and defined in robot-config.cpp — do NOT redeclare them here

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL STATE
// ═══════════════════════════════════════════════════════════════════════════

StateMachine g_state;

// Control loop timing
const double DT = 0.02; // 20ms = 50 Hz

// ═══════════════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

// Calibrate sensors at match start
void calibrate()
{
    Inertial.calibrate();
    TrackingWheel.resetPosition(0);
    while (Inertial.isCalibrating())
    {
        wait(20, msec);
    }
    g_state.odom.reset(0, 0, Inertial.heading(rotationUnits::deg));
}

// Normalize angle to -180..180
double normalizeAngle(double a)
{
    while (a > 180)
        a -= 360;
    while (a < -180)
        a += 360;
    return a;
}

// Drive tank with left/right velocity (0..100 %)
void setDriveVel(double leftPct, double rightPct)
{
    LeftDrive.spin(fwd, leftPct, pct);
    RightDrive.spin(fwd, rightPct, pct);
}

void stopDrive()
{
    LeftDrive.stop();
    RightDrive.stop();
}

// Turn in place using PID
void turnToHeading(double targetDeg, double tolerance = 2.0, double timeout_ms = 5000)
{
    PID turnPID(TURN_KP, TURN_KI, TURN_KD);
    // target=0: we feed the normalized error as "current", so PID computes (0 - error).
    // Negate the output to get: positive error → positive output → right turn.
    turnPID.setTarget(0.0);
    vex::timer turnTimer;
    turnTimer.resetTimer();
    double current;

    while (true)
    {
        current = Inertial.heading(rotationUnits::deg);
        double error = normalizeAngle(targetDeg - current);
        double output = -turnPID.compute(error, DT);

        setDriveVel(-output, output);

        if (fabs(error) < tolerance)
            break;
        if (turnTimer.time(timeUnits::msec) > timeout_ms)
            break;
        wait(20, msec);
    }
    stopDrive();
}

// ═══════════════════════════════════════════════════════════════════════════
// AUTONOMOUS — Isolation Phase
// ═══════════════════════════════════════════════════════════════════════════

void auto_Isolation()
{
    calibrate();

// State machine: SEARCH → COLLECT → SCORE
        g_state.state = RobotState::SEARCH;

        bool collectRequested = false;

    while (true)
    {
        // Update odometry
        double trackingDeg = TrackingWheel.position(rotationUnits::deg);
        double headingDeg = Inertial.heading(rotationUnits::deg);
        g_state.odom.update(trackingDeg, headingDeg);

        switch (g_state.state)
        {

        case RobotState::SEARCH:
        {
            // Use pure pursuit to go to nearest collect zone
            Waypoint *zones = g_state.COLLECT_ZONES;
            int bestIdx = 0;
            double bestDist = 1e9;
            for (int i = 0; i < 3; i++)
            {
                double d = g_state.pp.distTo(zones[i].x, zones[i].y);
                if (d < bestDist)
                {
                    bestDist = d;
                    bestIdx = i;
                }
            }

            Waypoint target = zones[bestIdx];
            target.slow = true;
            g_state.pp.loadPath(&target, 1);

            // driveTo() returns true when within tolerance
            bool arrived = g_state.pp.driveTo(target.x, target.y, 3.0);
            if (arrived)
            {
                g_state.state = RobotState::COLLECT;
            }

            // Use driveTo() output — it stores velocities via compute()
            double lv = g_state.pp.getLastLeftV();
            double rv = g_state.pp.getLastRightV();
            // Convert in/s to % (max ~24 in/s at 100%)
            setDriveVel(lv / BASE_SPEED * 100.0, rv / BASE_SPEED * 100.0);
            break;
        }

        case RobotState::COLLECT:
        {
            // Run intake, drive slowly forward
            Intake.spin(fwd, 70, pct);
            setDriveVel(10, 10);

            // Use a fixed collection time — the intake catches balls as we drive
            // collectRequested starts false, gets set once when we enter COLLECT
            if (!collectRequested) {
                collectRequested = true;
            }

            // After 3 seconds of collecting, transition to goal
            // The Jetson AI detects balls and the intake physically collects them
            // We use a time-based estimate since detectionCount doesn't tell us
            // whether a ball was actually ingested vs just seen
            static double collectTimer = 0;
            collectTimer += DT;

            if (collectTimer >= 3.0) {
                collectTimer = 0;
                collectRequested = false;
                Intake.stop();
                g_state.state = RobotState::DRIVE_TO_GOAL;
            }
            break;
        }

        case RobotState::DRIVE_TO_GOAL:
        {
            // Go to closest goal using pure pursuit
            Waypoint goals[2] = {g_state.SCORE_LEFT, g_state.SCORE_RIGHT};
            int bestIdx = 0;
            double bestDist = 1e9;
            for (int i = 0; i < 2; i++)
            {
                double d = g_state.pp.distTo(goals[i].x, goals[i].y);
                if (d < bestDist)
                {
                    bestDist = d;
                    bestIdx = i;
                }
            }

            Waypoint target = goals[bestIdx];
            target.slow = true;
            g_state.pp.loadPath(&target, 1);

            bool arrived = g_state.pp.driveTo(target.x, target.y, 3.0);
            if (arrived)
            {
                g_state.state = RobotState::SCORE;
            }

            // Use driveTo() output for wheel velocities
            double lv = g_state.pp.getLastLeftV();
            double rv = g_state.pp.getLastRightV();
            setDriveVel(lv / BASE_SPEED * 100.0, rv / BASE_SPEED * 100.0);
            break;
        }

        case RobotState::SCORE:
        {
            // Back up and dump balls
            setDriveVel(-15, -15);
            wait(500, msec);
            stopDrive();

            // Score
            Intake.spin(fwd, 100, pct);
            wait(2, sec);
            Intake.stop();

            // Park
            g_state.state = RobotState::PARK;
            break;
        }

        case RobotState::PARK:
        {
            Waypoint park = g_state.PARK_POS;
            park.slow = true;
            g_state.pp.loadPath(&park, 1);
            bool arrived = g_state.pp.driveTo(park.x, park.y, 3.0);
            if (arrived)
                stopDrive();

            // Done — stay here
            wait(5, sec);
            return;
        }

        default:
            stopDrive();
            break;
        }

        // Debug — print to Brain screen
        BrainScreen.clearScreen();
        BrainScreen.printAt(0, 20, "State: %s", g_state.stateName());
        BrainScreen.printAt(0, 50, "X: %.1f  Y: %.1f  H: %.1f",
                          g_state.odom.x(), g_state.odom.y(), g_state.odom.theta());

        wait(20, msec);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// DRIVER CONTROL — improved with input curves and brake mode
// ═══════════════════════════════════════════════════════════════════════════

// Expo drive curve — smooths out raw controller input
// curveInput: applies y = a * x^3 + (1-a) * x with sign preserved
static double applyExpoCurve(double raw, double a = 0.5) {
    if (raw >= 0) return a * raw * raw * raw + (1 - a) * raw;
    else return -(a * (-raw) * (-raw) * (-raw) + (1 - a) * (-raw));
}

void usercontrol()
{
    controller Controller = controller();

    // Track previous button states for edge detection
    bool prevR1 = false, prevR2 = false, prevL1 = false, prevL2 = false;
    bool prevA = false, prevB = false, prevX = false, prevY = false;
    bool intakeActive = false;

    // Default brake mode: hold
    LeftDrive.setBrake(brakeType::hold);
    RightDrive.setBrake(brakeType::hold);

    while (true)
    {
        // ===== DRIVETRAIN — arcade with expo curve =====
        int leftY  = Controller.Axis3.position();      // forward/back
        int rightX = Controller.Axis4.position();       // turn
        int leftX  = Controller.Axis1.position();       // strafe (if needed)

        // Apply expo curve for smoother feel
        double fwd = applyExpoCurve(leftY / 127.0, 0.5) * 100;
        double turn = applyExpoCurve(rightX / 127.0, 0.5) * 80;

        // Tank split: forward is same, turn splits left/right
        double leftOut  = fwd + turn;
        double rightOut = fwd - turn;

        LeftDrive.spin(fwdType::fwd, leftOut, pct);
        RightDrive.spin(fwdType::fwd, rightOut, pct);

        // ===== INTAKE — R1 = intake in, R2 = intake out =====
        bool r1 = Controller.ButtonR1.pressing();
        bool r2 = Controller.ButtonR2.pressing();

        if (r1 && !prevR1) {
            intakeActive = !intakeActive;  // Toggle intake on/off
        }
        prevR1 = r1;
        prevR2 = r2;

        if (intakeActive) {
            Intake.spin(fwd, 70, pct);
        } else if (r2) {
            Intake.spin(fwd, -70, pct);  // Outtake
        } else {
            Intake.stop(brakeType::hold);
        }

        // ===== HOOD — L1/L2 for manual hood control =====
        bool l1 = Controller.ButtonL1.pressing();
        bool l2 = Controller.ButtonL2.pressing();

        if (l1) {
            Hood.spin(fwd, 60, pct);
        } else if (l2) {
            Hood.spin(fwd, -60, pct);
        } else {
            Hood.stop(brakeType::hold);
        }

        // ===== FLYWHEEL — A button toggles on/off =====
        bool a = Controller.ButtonA.pressing();
        if (a && !prevA) {
            static bool flywheelOn = false;
            flywheelOn = !flywheelOn;
            if (flywheelOn) {
                Flywheel.spin(fwd, 100, pct);
            } else {
                Flywheel.stop(brakeType::coast);
            }
        }
        prevA = a;

        // ===== BRAKE MODE TOGGLE — Y button =====
        bool y = Controller.ButtonY.pressing();
        if (y && !prevY) {
            static bool holdMode = true;
            holdMode = !holdMode;
            if (holdMode) {
                LeftDrive.setBrake(brakeType::hold);
                RightDrive.setBrake(brakeType::hold);
            } else {
                LeftDrive.setBrake(brakeType::coast);
                RightDrive.setBrake(brakeType::coast);
            }
        }
        prevY = y;

        // ===== DEBUG — Brain screen =====
        BrainScreen.clearScreen();
        BrainScreen.printAt(0, 0, "Intake: %s  Flywheel: %s",
            intakeActive ? "ON" : "OFF",
            Flywheel.velocity(pct) > 5 ? "SPIN" : "OFF");
        BrainScreen.printAt(0, 30, "X: %.1f  Y: %.1f  H: %.1f",
            g_state.odom.x(), g_state.odom.y(), g_state.odom.theta());

        wait(20, msec);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN
// ═══════════════════════════════════════════════════════════════════════════

int main()
{
    // Request initial frame from Jetson AI vision
    jetson_comms.request_map();

    // Register competition callbacks
    Competition.autonomous(auto_Isolation);
    Competition.drivercontrol(usercontrol);

    // Default: run driver control until competition starts
    while (!Competition.isEnabled())
    {
        wait(20, msec);
    }
}
