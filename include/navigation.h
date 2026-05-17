

#ifndef NAVIGATION_H
#define NAVIGATION_H

#if __has_include(<cmath>)
#include <cmath>
#elif __has_include(<math.h>)
#include <math.h>
#else
#error "Missing math header"
#endif

#if __has_include(<cstdint>)
#include <cstdint>
#elif __has_include(<stdint.h>)
#include <stdint.h>
#else
#error "Missing stdint header"
#endif

#if defined(__has_include)
#if __has_include("vex_aivision.h")
#include "vex_aivision.h"
#elif __has_include(<vex_aivision.h>)
#include <vex_aivision.h>
#elif __has_include("vex.h")
#include "vex.h"
#elif __has_include(<vex.h>)
#include <vex.h>
#else
// Minimal VEX stubs for environments without VEX headers available.
namespace vex
{
    enum directionType
    {
        fwd,
        rev
    };
}
namespace ai
{
    class jetson
    {
    public:
        int get_data(void *) { return 0; }
    };
}
using PortName = int;
constexpr PortName PORT1 = 1;
constexpr PortName PORT2 = 2;
constexpr PortName PORT3 = 3;
constexpr PortName PORT4 = 4;
constexpr PortName PORT5 = 5;
constexpr PortName PORT6 = 6;
constexpr PortName PORT7 = 7;
constexpr PortName PORT8 = 8;
constexpr PortName PORT9 = 9;
constexpr PortName PORT10 = 10;
constexpr PortName PORT11 = 11;
constexpr PortName PORT12 = 12;
constexpr PortName PORT13 = 13;
constexpr PortName PORT14 = 14;
constexpr PortName PORT15 = 15;
constexpr PortName PORT16 = 16;
constexpr PortName PORT17 = 17;
constexpr PortName PORT18 = 18;
constexpr PortName PORT19 = 19;
constexpr PortName PORT20 = 20;

struct AI_RECORD
{
    int detectionCount = 0;
};
#endif
#else
#include "vex.h"
#endif

// ═══════════════════════════════════════════════════════════════════════════
// CONFIG — update these for your robot
// ═══════════════════════════════════════════════════════════════════════════

// Drive motors (3 per side — adjust ports as needed)
#define LEFT_DRIVE_PORTS PORT1, PORT2, PORT3
#define RIGHT_DRIVE_PORTS PORT11, PORT12, PORT13

// Tracking wheel (2" omni, direct-drive rotation sensor)
#define TRACKING_WHEEL_PORT PORT20

// Inertial sensor (heading)
#define INERTIAL_PORT PORT7

// Mechanism motors
#define INTAKE_MOTOR_PORT PORT4 // 1st stage intake
#define HOOD_MOTOR_PORT PORT5   // hood

// Robot dimensions (inches)
#define DRIVE_WHEEL_DIAMETER 3.25   // drive wheels (inches)
#define TRACKING_WHEEL_DIAMETER 2.0 // tracking omni (inches)
#define TRACK_WIDTH 12.0            // center-to-center wheel spread (inches)

// Pure pursuit
#define LOOKAHEAD_DISTANCE 12.0 // inches ahead to look
#define BASE_SPEED 24.0         // inches/sec max

// PID gains — TUNE THESE for your robot
#define TURN_KP 1.2
#define TURN_KI 0.0
#define TURN_KD 0.1

#define DRIVE_KP 1.0
#define DRIVE_KI 0.0
#define DRIVE_KD 0.05

// ═══════════════════════════════════════════════════════════════════════════
// MATH CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════

#define PI 3.14159265359
#define TWO_PI 6.28318530718

// Degrees per radian
#define DEG_PER_RAD 57.2957795131
// Radians per degree
#define RAD_PER_DEG 0.0174532925199

// ═══════════════════════════════════════════════════════════════════════════
// ODOMETRY
// ═══════════════════════════════════════════════════════════════════════════

class Odometry
{
public:
    // Current position (x, y in inches; theta in degrees)
    double _x, _y, _theta;
    // Previous tracking wheel position (degrees)
    double _prevTracking;
    // Inertial-based theta (more reliable than tracking wheel diff)
    double _thetaInertial;

    Odometry() : _x(0), _y(0), _theta(0), _prevTracking(0), _thetaInertial(0) {}

    // Distance per degree of tracking wheel (inches/degree)
    static double distPerDeg()
    {
        return (PI * TRACKING_WHEEL_DIAMETER) / 360.0;
    }

    // Update position using tracking wheel delta and inertial heading
    // Call this every control loop tick (e.g., every 20ms)
    void update(double trackingDegrees, double inertialHeading)
    {
        // Delta tracking wheel (degrees)
        double delta = trackingDegrees - _prevTracking;
        _prevTracking = trackingDegrees;

        // Distance traveled (inches)
        double dist = delta * distPerDeg();

        // Normalize theta to -180..180
        _thetaInertial = inertialHeading;
        _theta = _thetaInertial;

        // Update x, y using robot's heading
        double thetaRad = _theta * RAD_PER_DEG;
        _x += dist * cos(thetaRad);
        _y += dist * sin(thetaRad);
    }

    void reset(double x = 0, double y = 0, double theta = 0)
    {
        _x = x;
        _y = y;
        _theta = theta;
        _prevTracking = 0;
        _thetaInertial = theta;
    }

    double x() const { return _x; }
    double y() const { return _y; }
    double theta() const { return _theta; }
};

// ═══════════════════════════════════════════════════════════════════════════
// PID CONTROLLER
// ═══════════════════════════════════════════════════════════════════════════

class PID
{
public:
    double _kp, _ki, _kd;
    double _integral, _prevError;
    double _target;
    bool _firstReading;

    PID(double kp = 1.0, double ki = 0.0, double kd = 0.0)
        : _kp(kp), _ki(ki), _kd(kd),
          _integral(0), _prevError(0), _target(0), _firstReading(true) {}

    void setGains(double kp, double ki, double kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    void setTarget(double target) { _target = target; }

    // Returns motor output (capped at -100..100 for percent units)
    double compute(double current, double dt = 0.02)
    {
        double error = _target - current;

        if (_firstReading)
        {
            _prevError = error;
            _firstReading = false;
        }

        _integral += error * dt;
        // Anti-windup: clamp integral
        if (_ki > 0)
        {
            double maxI = 50.0 / _ki;
            if (_integral > maxI)
                _integral = maxI;
            if (_integral < -maxI)
                _integral = -maxI;
        }

        double derivative = (error - _prevError) / dt;
        double output = _kp * error + _ki * _integral + _kd * derivative;

        _prevError = error;
        return output;
    }

    void reset()
    {
        _integral = 0;
        _prevError = 0;
        _firstReading = true;
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// PURE PURSUIT
// ═══════════════════════════════════════════════════════════════════════════

struct Waypoint
{
    double x, y; // inches
    bool slow;   // true = approach slowly (for scoring, intake, etc.)
    Waypoint(double x_ = 0, double y_ = 0, bool slow_ = false)
        : x(x_), y(y_), slow(slow_) {}
};

class PurePursuit
{
public:
    Odometry &_odom;
    double _lookahead;
    double _baseSpeed;

    // Active waypoints (path)
    Waypoint *_path;
    int _pathLen;
    int _currentIdx;

    // PID for heading correction
    PID _headingPID;

    PurePursuit(Odometry &odom)
        : _odom(odom),
          _lookahead(LOOKAHEAD_DISTANCE),
          _baseSpeed(BASE_SPEED),
          _path(NULL), _pathLen(0), _currentIdx(0),
          _headingPID(TURN_KP, TURN_KI, TURN_KD) {}

    // Load a path (caller keeps pointer alive)
    void loadPath(Waypoint *path, int len)
    {
        _path = path;
        _pathLen = len;
        _currentIdx = 0;
    }

    // Reset to beginning of path
    void reset() { _currentIdx = 0; }

    // Are we at the end of the path?
    bool done() const { return _currentIdx >= _pathLen; }

    // Distance from robot to a point
    double distTo(double px, double py) const
    {
        double dx = px - _odom.x();
        double dy = py - _odom.y();
        return sqrt(dx * dx + dy * dy);
    }

    // Angle from robot to a point (degrees, -180..180)
    double angleTo(double px, double py) const
    {
        double dx = px - _odom.x();
        double dy = py - _odom.y();
        double angle = atan2(dy, dx) * DEG_PER_RAD;
        return angle;
    }

    // Normalize angle to -180..180
    static double normalizeAngle(double a)
    {
        while (a > 180)
            a -= 360;
        while (a < -180)
            a += 360;
        return a;
    }

    // Find the lookahead point on the path
    // Returns index of the waypoint past the lookahead circle
    int findLookaheadPoint()
    {
        // Start from current waypoint and look ahead
        for (int i = _currentIdx; i < _pathLen; i++)
        {
            if (distTo(_path[i].x, _path[i].y) >= _lookahead)
            {
                return i;
            }
        }
        return _pathLen - 1; // default to last waypoint
    }

    // Normalize angle to -π..π
    static double normalizeAngleRad(double a)
    {
        while (a > M_PI) a -= TWO_PI;
        while (a < -M_PI) a += TWO_PI;
        return a;
    }

    // Compute curvature to a target point
    // curvature = 1/radius (positive = turn left)
    double computeCurvature(double targetX, double targetY)
    {
        double robotX = _odom.x();
        double robotY = _odom.y();
        double robotTheta = _odom.theta() * RAD_PER_DEG;

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // Distance to target
        double L = sqrt(dx * dx + dy * dy);
        if (L < 0.001)
            return 0;

        // Angle to target in robot frame (radians)
        double alpha = atan2(dy, dx) - robotTheta;
        // Normalize to -π..π (avoids double-conversion bug)
        alpha = normalizeAngleRad(alpha);

        // Signed curvature: kappa = 2*sin(α)/L
        double kappa = 2.0 * sin(alpha) / L;

        // Determine turn direction from relative angle to target
        double angleToTarget = angleTo(targetX, targetY);
        double relAngle = normalizeAngle(angleToTarget - _odom.theta());
        return (relAngle > 0) ? fabs(kappa) : -fabs(kappa);
    }

    // Compute wheel velocities to follow path
    // Returns {leftVel, rightVel} in rpm
    void computeWheelVels(double &leftV, double &rightV)
    {
        if (done())
        {
            leftV = 0;
            rightV = 0;
            return;
        }

        int idx = findLookaheadPoint();
        Waypoint &wp = _path[idx];

        // Speed: slow down when waypoint says so or when close
        double speed = _baseSpeed;
        if (wp.slow)
            speed *= 0.4;
        double distRemaining = distTo(wp.x, wp.y);
        if (distRemaining < 6.0)
            speed *= 0.3;

        // Curvature to lookahead point
        double kappa = computeCurvature(wp.x, wp.y);

        // Tank steering: curvature-based velocity split
        // v_left  = v_avg * (1 + kappa * track_width/2)
        // v_right = v_avg * (1 - kappa * track_width/2)
        double trackHalf = TRACK_WIDTH / 2.0;
        double vAvg = speed * 1.0; // in/s → will be converted

        leftV = vAvg * (1.0 + kappa * trackHalf);
        rightV = vAvg * (1.0 - kappa * trackHalf);

        // Advance index when close to current waypoint
        if (distRemaining < 3.0 && idx == _currentIdx)
        {
            _currentIdx++;
        }
    }

    // Direct driveTo — single target with PID
    // Returns true when within tolerance
    bool driveTo(double targetX, double targetY, double tolerance = 2.0)
    {
        double dist = distTo(targetX, targetY);
        if (dist < tolerance)
            return true;

        double angle = angleTo(targetX, targetY);
        double relAngle = normalizeAngle(angle - _odom.theta());

        // Heading correction
        _headingPID.setTarget(_odom.theta() + relAngle);
        double turnCmd = _headingPID.compute(_odom.theta());

        // Speed based on distance
        double speedCmd = DRIVE_KP * dist;
        if (speedCmd > _baseSpeed)
            speedCmd = _baseSpeed;
        if (speedCmd < 3.0)
            speedCmd = 3.0;

        double leftV = speedCmd - turnCmd;
        double rightV = speedCmd + turnCmd;

        // Apply to motors (caller does this)
        // leftDrive.spin(fwd, leftV, rpm);
        // rightDrive.spin(fwd, rightV, rpm);

        // Store for retrieval
        _lastLeftV = leftV;
        _lastRightV = rightV;

        return false;
    }

    double getLastLeftV() const { return _lastLeftV; }
    double getLastRightV() const { return _lastRightV; }

private:
    double _lastLeftV = 0, _lastRightV = 0;
};

// ═══════════════════════════════════════════════════════════════════════════
// SERIAL COMMS — Jetson Nano ↔ V5
// Use VEX's built-in ai::jetson class (vex_aivision.h)
// Handles full protocol: sync bytes AA 55 CC 33, CRC32, threaded receive
// The Jetson sends detection data; V5 requests it with request_map()
// ═══════════════════════════════════════════════════════════════════════════

class JetsonComms
{
public:
    JetsonComms() : _detectionCount(0), _connected(false) {}

    bool init()
    {
        _connected = true;
        return true;
    }

    int update()
    {
        if (!_connected)
        {
            return 0;
        }

        int32_t result = _jetson.get_data(&_latest);
        if (result <= 0)
        {
            _detectionCount = 0;
            return 0;
        }

        _detectionCount = _latest.detectionCount;
        return _detectionCount;
    }

    bool getDetection(int index, AI_RECORD &out)
    {
        if (index < 0 || index >= _latest.detectionCount)
        {
            return false;
        }
        out = _latest;
        return true;
    }

    int detectionCount() const { return _detectionCount; }
    bool isConnected() const { return _connected; }
    void close() {}

private:
    ai::jetson _jetson;
    AI_RECORD _latest;
    int _detectionCount;
    bool _connected;
};

// ═══════════════════════════════════════════════════════════════════════════
// STATE MACHINE — SEARCH → COLLECT → SCORE
// ═══════════════════════════════════════════════════════════════════════════

enum class RobotState
{
    IDLE,
    SEARCH,
    COLLECT,
    DRIVE_TO_GOAL,
    SCORE,
    PARK
};

class StateMachine
{
public:
    RobotState state = RobotState::IDLE;
    Odometry odom;
    PurePursuit pp;
    JetsonComms jetson;

    // Waypoints (field coords — verify with game manual)
    Waypoint COLLECT_ZONES[3];
    Waypoint SCORE_LEFT;
    Waypoint SCORE_RIGHT;
    Waypoint PARK_POS;

    StateMachine() : odom(), pp(odom)
    {
        // Field is 369x369 cm. Origin at center (0,0).
        // X-East (+X = east), Y-North (+Y = north), 0° = North
        // Ball zones: areas where balls are distributed around the field perimeter
        // Starting zone for worker (3177B): near center-ish
        COLLECT_ZONES[0] = Waypoint(0, 80);      // north center-ish
        COLLECT_ZONES[1] = Waypoint(70, 40);     // northeast
        COLLECT_ZONES[2] = Waypoint(-70, 40);    // northwest
        SCORE_LEFT  = Waypoint(-70, 120, true);   // corner goal NW (slow approach)
        SCORE_RIGHT = Waypoint( 70, 120, true);   // corner goal NE (slow approach)
        PARK_POS    = Waypoint(0, 0);            // return to center
    }

    void reset()
    {
        odom.reset();
        pp.reset();
        state = RobotState::IDLE;
    }

    const char *stateName() const
    {
        switch (state)
        {
        case RobotState::IDLE:
            return "IDLE";
        case RobotState::SEARCH:
            return "SEARCH";
        case RobotState::COLLECT:
            return "COLLECT";
        case RobotState::DRIVE_TO_GOAL:
            return "DRIVE_TO_GOAL";
        case RobotState::SCORE:
            return "SCORE";
        case RobotState::PARK:
            return "PARK";
        default:
            return "UNKNOWN";
        }
    }
};

#endif // NAVIGATION_H
