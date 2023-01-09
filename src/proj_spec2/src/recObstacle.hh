#include "obstacle.hh"
class recObstacle: public obstacle
{
    float posX[4];
    float posY[4];

    float length;
    float height;

public:
    recObstacle(float x, float y, float l, float h);
    bool checkCollision(float x, float y);
};