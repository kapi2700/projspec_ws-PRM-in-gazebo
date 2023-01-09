#include <vector>
#include <fstream>

#include "recObstacle.hh"
#include "cirObstacle.hh"

using namespace std;

class world
{
public:
    float maxGridX;
    float maxGridY;
    std::vector<obstacle*> obstacles;

public:
    world(float X, float Y)
    {
        maxGridX=X;
        maxGridY=Y;
    }
};