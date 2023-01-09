#pragma once
#include "obstacle.hh"

class cirObstacle: public obstacle
{
    float posX;
    float posY;
    float radius;
public:
    cirObstacle(float x, float y, float r);
    bool checkCollision(float x, float y);
};