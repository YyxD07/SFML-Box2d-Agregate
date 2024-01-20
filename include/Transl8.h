#ifndef Transl8_H
#define Transl8_H


#include "Graphics.hpp"
#include "../box2d-main/include/box2d/box2d.h"

#include <cmath>
#include <memory>

#include "ConstantSettings.h"

#include "EdgyStripe.h"
#include "EdgyFan.h"


namespace Transl8
{
// Translates 2d vector from b2 to sfml (size scaled by constant settings scale)
sf::Vector2f vec2(const b2Vec2& xy);

// Translates 2d vector from sfml to b2 (size decreased by constant settings scale)
b2Vec2 vec2(const sf::Vector2f& xy);

/* Takes sfml object and returns its rotation in radians for use with box2d. Box2d positive angle
is in counterclockwise direction while sfml positive angle is in clockwise direction, angle sign must
thus change.*/ // untested!
float getRotation(const sf::Transformable& object);

/* Takes box2d object and returns its rotation in degrees for use with sfml. Box2d positive angle
is in counterclockwise direction while sfml positive angle is in clockwise direction, angle sign must
thus change.*/
float getRotation(b2Body* object);

/*Takes box2d polygon and returns scaled (according to universalConstants::kScaling) B2ToSf::EdgyFan.
Note: b2PolygonShape has no rotation. Use b2Body rotation for setting EdgyFan rotation */
B2ToSf::EdgyFan convexPolygon(const b2PolygonShape& b2_poligon);

/* Takes two vertex box2d edge and returns 4 vertex sfml polygon. Each edge vertex has two corresponding vertices, translated
in normal direction for + and - half of chosen line width*/
B2ToSf::EdgyFan line (const b2EdgeShape& b2_edge, float width = 1);

// Makes a SFML circle from a box2d circle
B2ToSf::EdgyFan circle(const b2CircleShape& b2_circle, std::size_t point_count = 30);

// Makes SFML representation of a box2d chain
B2ToSf::EdgyStripe chainShape(const b2ChainShape& b2_chain, float width = 1);

/***Wrapper functions for four box2d to sfml shape translation functions that return unique_ptr-s to new heap objects.
All four previous functions should be merged with these four so they actually return new objects by default in future...***/


std::unique_ptr<B2ToSf::EdgyFan> b2ToSFMLPolygon(const b2Shape& b2_polygon);
std::unique_ptr<B2ToSf::EdgyFan> b2ToSFMLEdge(const b2Shape& b2_edge, float width = 1);
std::unique_ptr<B2ToSf::EdgyFan> b2ToSFMLCircle(const b2Shape& b2_circle, std::size_t point_count = 30);
std::unique_ptr<B2ToSf::EdgyStripe> b2ToSFMLChain(const b2Shape& b2_chain, float width = 1);







}// namespace Transl8

#endif // Transl8_H
