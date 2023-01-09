#include "recObstacle.hh"

recObstacle::recObstacle(float x, float y, float l, float h)    //rectangle.jpg
{
    posX[0]=x;
    posY[0]=y;

    posX[1]=x+l;
    posY[1]=y;

    posX[2]=x+l;
    posY[2]=y+h;

    posX[3]=x;
    posY[3]=y+h;
}

bool recObstacle::checkCollision(float x, float y)
{
    if((x>posX[0]-COLLISIONMARGIN) && (x<posX[1]+COLLISIONMARGIN))
        if((y>posY[0]-COLLISIONMARGIN)&&(y<posY[2]+COLLISIONMARGIN))
            return false;
    return true;
}