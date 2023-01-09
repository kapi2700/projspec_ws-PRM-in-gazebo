#include "cirObstacle.hh"

cirObstacle::cirObstacle(float x, float y, float r)
{
    posX=x;
    posY=y;
    radius=r;
}

bool cirObstacle::checkCollision(float x, float y)
{
    float odleglosc;
    odleglosc=(posX-x)*(posX-x)+(posY-y)*(posY-y);
    if((radius+COLLISIONMARGIN)*(radius+COLLISIONMARGIN)>odleglosc) //unikam pierwiastkownia
        return false;
    return true;
}