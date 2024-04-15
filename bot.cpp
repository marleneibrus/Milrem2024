#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>

using namespace std;

struct CheckPoint {
    int x;
    int y;
    CheckPoint(int _x, int _y) : x(_x), y(_y) {}
};

class CheckpointManager {
private:
    std::vector<CheckPoint> listCheckPoint;
    bool firstTurn;

public:
    CheckpointManager() : firstTurn(true) {}

    void checkCP(int x, int y) {
        bool found = false;
        for (const auto& checkpoint : listCheckPoint) {
            if (checkpoint.x == x && checkpoint.y == y) {
                std::cerr << "OH !" << std::endl;
                firstTurn = false;
                found = true;
                break;
            }
        }
        if (!found) {
            std::cerr << "push" << std::endl;
            listCheckPoint.push_back({x, y});
        }
    }
    bool bestBoost(int x, int y) { // Find the best distance to use boost.
        if (!firstTurn) {
            double maxDistance = -INFINITY;
            CheckPoint maxCP(0, 0);

            for (const auto& el1 : listCheckPoint) {
                for (const auto& el2 : listCheckPoint) {
                    double d = distance(el1.x, el2.x, el1.y, el2.y);
                    if (d > maxDistance) {
                        maxDistance = d;
                        maxCP = el2;
                    }
                }
            }
            std::cerr << "Max CP: " << maxCP.x << ", " << maxCP.y << std::endl;
            if (maxCP.x == x && maxCP.y == y) {
                return true;
            }
        }
        return false;
    }
    private:
        double distance(int x1, int x2, int y1, int y2) {
            return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        }
};

double distance(int x1, int x2, int y1, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool useShield(int opponent_x,int opponent_y, int x, int y, int next_checkpoint_angle) {
    double angle = atan2(opponent_y - y, opponent_x - x);
    double angleDeg = (angle * 180) / M_PI;
    double distanceToOpponent = distance(opponent_x, y, opponent_y, y);

    if (distanceToOpponent <= 500) {
        return true;
    }
    return false;
}

constexpr double givenAngle = 60; // A treshold for correcting thrust
constexpr double k = 2.0;  
constexpr double h = 3.0;
bool smallangle = false;

class PodController {
    public:
    double calculateThrust(double angle, double distanceToCheckpoint, double checkPointRadius) {
        double thrust = 100; // Default thrust value
        if (angle < givenAngle) {
            thrust = 100;
            smallangle = true;
        } else {
            // Calculate factors for thrust adjustment
            double angleFactor = max(0.0, min(1.0, 1.0 - angle / 90.0));
            double distanceFactor = max(0.0, min(1.0, distanceToCheckpoint / (k * checkPointRadius)));
            
            thrust *= min(angleFactor, distanceFactor);
        }
        return thrust;
    }

    double adjustRotationX(double targetDirectionX, double targetDirectionY, double podSpeedX, double podSpeedY) {
        double currentDirectionX = podSpeedX;
        double currentDirectionY = podSpeedY;
        double errorX = targetDirectionX - currentDirectionX;
        double errorY = targetDirectionY - currentDirectionY;

        // Proportional control: adjust rotation proportionally to the error
        double proportionalTerm = 0.2; // Chosen based on heuristics
        double rotationAdjustmentX = proportionalTerm * errorX;

        // Adjust rotation of the bot
        double adjustedDirectionX = currentDirectionX + rotationAdjustmentX;
        return adjustedDirectionX;
    }

    double adjustRotationY(double targetDirectionY, double podSpeedX, double podSpeedY) {
        double currentDirectionX = podSpeedX;
        double currentDirectionY = podSpeedY;
        double errorY = targetDirectionY - currentDirectionY;

        // Proportional control: adjust rotation proportionally to the error
        double proportionalTerm = 0.2; // Chosen based on heuristics.
        double rotationAdjustmentY = proportionalTerm * errorY;

        // Adjust rotation of the bot
        double adjustedDirectionY = currentDirectionY + rotationAdjustmentY;
        return adjustedDirectionY;
    }
};

int main() {
    CheckpointManager cpm;
    PodController pc;
    bool boost = false;
    int lastX = 0, lastY = 0;

    // game loop
    while (1) {
        int x;
        int y;
        int thrust;
        int next_checkpoint_x; // x position of the next check point
        int next_checkpoint_y; // y position of the next check point
        int next_checkpoint_dist; // distance to the next checkpoint
        int next_checkpoint_angle; // angle between your pod orientation and the direction of the next checkpoint
        
        cin >> x >> y >> next_checkpoint_x >> next_checkpoint_y >> next_checkpoint_dist >> next_checkpoint_angle; cin.ignore();
        int opponent_x;
        int opponent_y;
        cin >> opponent_x >> opponent_y; cin.ignore();

    
        double distance_to_opponent = sqrt(pow(opponent_x - x, 2) + pow(opponent_y - y, 2));
        double opponent_relative_angle = atan2(opponent_y - y, opponent_x - x) * 180 / M_PI;

        string outputthrust = "";
        // Example strategy: Slow down if the opponent is too close

        thrust = pc.calculateThrust(abs(next_checkpoint_angle), next_checkpoint_dist, 300);
        int adjusted_next_x = pc.adjustRotationX(next_checkpoint_x, next_checkpoint_y, x, y);
        int adjusted_next_y = pc.adjustRotationY(next_checkpoint_y, x, y);
        outputthrust = to_string(thrust);

        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"

        if (useShield(opponent_x, opponent_y, x, y, next_checkpoint_angle)) {
            // outputthrust = "SHIELD"; Not using the shield right now
        } else if (!boost && cpm.bestBoost(next_checkpoint_x, next_checkpoint_y) && abs(next_checkpoint_angle) < 2) {
            outputthrust = "BOOST";
            boost = true;
        } else if (abs(next_checkpoint_angle) > 80) {
            outputthrust = "10";
        }

        if (next_checkpoint_x != lastX && next_checkpoint_y != lastY) {
            cpm.checkCP(next_checkpoint_x, next_checkpoint_y);
            lastX = next_checkpoint_x;
            lastY = next_checkpoint_y;
        }

        cout << adjusted_next_x << " " << adjusted_next_y << " " << outputthrust << endl;
    }
}