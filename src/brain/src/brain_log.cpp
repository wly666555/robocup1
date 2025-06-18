#include "brain_log.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"

BrainLog::BrainLog(Brain *argBrain) : enabled(false), brain(argBrain), rerunLog("robocup")
{
    if (!brain->config->rerunLogEnable)
    {
        enabled = false;
        return;
    }

    rerun::Error err = rerunLog.connect(brain->config->rerunLogServerAddr);
    if (err.is_err())
    {
        prtErr("Connect rerunLog server failed: " + err.description);
        enabled = false;
        return;
    }

    enabled = true;
}

void BrainLog::setTimeNow()
{
    if (!enabled)
        return;

    setTimeSeconds(brain->get_clock()->now().seconds());
}

void BrainLog::setTimeSeconds(double seconds)
{
    if (!enabled)
        return;

    rerunLog.set_time_seconds("time", seconds);
}

void BrainLog::logStatics()
{
    if (!enabled)
        return;

    FieldDimensions &fieldDimensions = brain->config->fieldDimensions;
    rerun::Collection<rerun::Vec2D> borders = {{-fieldDimensions.length / 2, -fieldDimensions.width / 2}, {fieldDimensions.length / 2, -fieldDimensions.width / 2}, {fieldDimensions.length / 2, fieldDimensions.width / 2}, {-fieldDimensions.length / 2, fieldDimensions.width / 2}, {-fieldDimensions.length / 2, -fieldDimensions.width / 2}};
    rerun::Collection<rerun::Vec2D> centerLine = {{0, -fieldDimensions.width / 2}, {0, fieldDimensions.width / 2}};
    rerun::Collection<rerun::Vec2D> leftPenalty = {{-fieldDimensions.length / 2, fieldDimensions.penaltyAreaWidth / 2}, {-(fieldDimensions.length / 2 - fieldDimensions.penaltyAreaLength), fieldDimensions.penaltyAreaWidth / 2}, {-(fieldDimensions.length / 2 - fieldDimensions.penaltyAreaLength), -fieldDimensions.penaltyAreaWidth / 2}, {-fieldDimensions.length / 2, -fieldDimensions.penaltyAreaWidth / 2}};
    rerun::Collection<rerun::Vec2D> rightPenalty = {{fieldDimensions.length / 2, fieldDimensions.penaltyAreaWidth / 2}, {(fieldDimensions.length / 2 - fieldDimensions.penaltyAreaLength), fieldDimensions.penaltyAreaWidth / 2}, {(fieldDimensions.length / 2 - fieldDimensions.penaltyAreaLength), -fieldDimensions.penaltyAreaWidth / 2}, {fieldDimensions.length / 2, -fieldDimensions.penaltyAreaWidth / 2}};
    rerun::Collection<rerun::Vec2D> leftGoal = {{-fieldDimensions.length / 2, fieldDimensions.goalAreaWidth / 2}, {-(fieldDimensions.length / 2 - fieldDimensions.goalAreaLength), fieldDimensions.goalAreaWidth / 2}, {-(fieldDimensions.length / 2 - fieldDimensions.goalAreaLength), -fieldDimensions.goalAreaWidth / 2}, {-fieldDimensions.length / 2, -fieldDimensions.goalAreaWidth / 2}};
    rerun::Collection<rerun::Vec2D> rightGoal = {{fieldDimensions.length / 2, fieldDimensions.goalAreaWidth / 2}, {(fieldDimensions.length / 2 - fieldDimensions.goalAreaLength), fieldDimensions.goalAreaWidth / 2}, {(fieldDimensions.length / 2 - fieldDimensions.goalAreaLength), -fieldDimensions.goalAreaWidth / 2}, {fieldDimensions.length / 2, -fieldDimensions.goalAreaWidth / 2}};

    vector<rerun::Vec2D> circle = {{fieldDimensions.circleRadius, 0}};
    for (int i = 0; i < 360; i++)
    {
        double r = fieldDimensions.circleRadius;
        double theta = (i + 1) * M_PI / 180;
        circle.push_back(rerun::Vec2D{r * cos(theta), r * sin(theta)});
    }

    rerunLog.log(
        "field/lines",
        rerun::LineStrips2D({borders, centerLine, leftPenalty, rightPenalty, leftGoal, rightGoal, rerun::Collection<rerun::Vec2D>(circle)})
            .with_colors({0xFFFFFFFF})
            .with_radii({0.01})
            .with_draw_order(0.0));

    rerun::Collection<rerun::Vec2D> xaxis = {{-2, 0}, {2, 0}};
    rerun::Collection<rerun::Vec2D> yaxis = {{0, 2}, {0, -2}};
    rerun::Collection<rerun::Vec2D> border2m = {{-2, -2}, {-2, 2}, {2, 2}, {2, -2}, {-2, -2}};
    rerun::Collection<rerun::Vec2D> border1m = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
    rerun::Collection<rerun::Vec2D> angle = {{3 * cos(0.2), 3 * sin(0.2)}, {0, 0}, {3 * cos(0.2), 3 * sin(-0.2)}};

    rerunLog.log(
        "robotframe/lines",
        rerun::LineStrips2D({xaxis, yaxis, border2m, border1m, angle})
            .with_colors({0xFFFFFFFF})
            .with_radii({0.005, 0.005, 0.005, 0.002, 0.002})
            .with_draw_order(0.0));
}

void BrainLog::prepare()
{
    if (!enabled)
        return;

    setTimeSeconds(0);
    setTimeNow();
    logStatics();
}

void BrainLog::logToScreen(string logPath, string text, u_int32_t color, double padding)
{
    if (!enabled)
        return;

    log(
        logPath,
        rerun::Boxes2D::from_mins_and_sizes({{-padding, -padding}}, {{brain->config->camPixX + padding, brain->config->camPixY + padding}})
            .with_labels({text})
            .with_colors({color}));
}