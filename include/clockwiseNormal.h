#ifndef CLOCKWISENORMAL_H
#define CLOCKWISENORMAL_H

#include "Graphics.hpp"
#include "box2d.h"
#include <cmath>



namespace B2ToSf
{
/***Functions return scaled (by clockenss) normal vector to a vector pointing form v1 to v2. Positive values of clockenss
return clockwise vector, negative values of clockenss return coutnerclockwise vector. ***/
sf::Vector2f clockwiseNormal(const sf::Vector2f& v1, const sf::Vector2f& v2, float clockness = 1);
b2Vec2 clockwiseNormal(const b2Vec2& v1, const b2Vec2& v2, float clockness = 1);

/***Functions return scaled (by clockenss) normal vector to a vector v. Positive values of clockenss
return clockwise vector, negative values of clockenss return coutnerclockwise vector. ***/

sf::Vector2f clockwiseNormal(const sf::Vector2f& v, float clockness = 1);
b2Vec2 clockwiseNormal(const b2Vec2& v, float clockness = 1);

/***Determines rotation of vector (v22-v21) in respect to vector (v12-v11).
1 means counter-clockwise rotation
0 means vectors are pararel
-1 means  clockwise rotation  ***/
int rotationClockness(const sf::Vector2f& v11, const sf::Vector2f& v12, const sf::Vector2f& v21, const sf::Vector2f& v22);
int rotationClockness(const b2Vec2& v11, const b2Vec2& v12, const b2Vec2& v21, const b2Vec2& v22);


/***Determines rotation of vector (v2-v00) in respect to vector (v1-v00).
1 means counter-clockwise rotation
0 means vectors are pararel
-1 means  clockwise rotation  ***/
int rotationClockness(const sf::Vector2f v00, const sf::Vector2f v1, const sf::Vector2f v2);
int rotationClockness(const b2Vec2 v00, const b2Vec2 v1, const b2Vec2 v2);


/***Determines rotation of vector (v2) in respect to vector (v1).
1 means counter-clockwise rotation
0 means vectors are pararel
-1 means  clockwise rotation  ***/
int rotationClockness(const sf::Vector2f v1, const sf::Vector2f v2);
int rotationClockness(const b2Vec2 v1, const b2Vec2 v2);

/***Extends corner in the direction: clockwise unit normal to vector ( adjecent1 pointing to corner) plus counter clockwise
unit normal to vector (adjacent2 pointing to corner ). Clockness can scale unit normals proportionally. Negative values reverse
direction of both normal vectors***/
sf::Vector2f extendCorner(const sf::Vector2f& corner, const sf::Vector2f& adjacent1, const sf::Vector2f& adjacent2, float clockness = 1);
}// namespace B2ToSf

#endif // CLOCKWISENORMAL_H
