#pragma once

#define COLLISIONMARGIN 0.2


class obstacle
{
public:
    bool virtual checkCollision(float x, float y)
    {
        return true;
    };
};