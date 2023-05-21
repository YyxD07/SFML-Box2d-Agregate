#include <iostream>
#include <SFML/Window.hpp>
#include <Graphics.hpp>
#include "box2d-main/include/box2d/box2d.h"

#include "constant settings.h"

#include "transl8.h"

#include "EdgyStripe.h"




int main()
{
    sf::Vector2f v00(0,0);
    sf::Vector2f v11(10, 0);
    sf::Vector2f v12(10,100);
    sf::Vector2f v21(15, 0);
    sf::Vector2f v22(160,0);

    sf::Vector2f vex(1,0);
    sf::Vector2f vey(0,1);
    sf::Vector2f vnex(-2,0);
    sf::Vector2f vney(0,-1);


    sf::Vector2f resultNorm = b2tosf::clockwiseNormal(vnex, vex);
    std::cout << resultNorm.x << " " << resultNorm.y << "\n";

    float resultClockness = b2tosf::rotationClockness(v21, v22);
    std::cout << resultClockness << "\n \n";

    b2Vec2 V11(10, 0);
    b2Vec2 V12(10,100);
    b2Vec2 V21(15, 0);
    b2Vec2 V22(160,0);

    b2Vec2 Vex(1,0);
    b2Vec2 Vey(0,1);
    b2Vec2 Vnex(-1,0);
    b2Vec2 Vney(0,-1);


    b2Vec2 Result = b2tosf::clockwiseNormal(Vney,-2);


    std::cout << Result.x << " " << Result.y << "\n";



    std::cout << "\n corner extension \n";
    sf::Vector2f cornerExt = b2tosf::extendCorner
    (sf::Vector2f(0,0)
    ,sf::Vector2f(1,0)
    ,sf::Vector2f(0,-1)
    ,1);
    std::cout << cornerExt.x << " " << cornerExt.y;

    return 0;
}
