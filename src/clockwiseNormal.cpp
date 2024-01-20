#include "clockwiseNormal.h"

namespace
{
// Template takes 2d vector with public .x and .y and returns scaled vector. Positive clockness is clockwise, negative is
//counterclockwise, +/- 1 returns normalized vector (length of 1), other values of clockness scale vector accordingly.
template <typename T>
T clockwiseNormalImp(const T v1, const T v2, float clockness)
{
    T vxy = v2;
    vxy -= v1;
    float vxy_length = std::sqrt(vxy.x * vxy.x + vxy.y * vxy.y); // calculating length of the vector
    vxy = T(vxy.x / vxy_length, vxy.y / vxy_length); // using calculated length to normalize the vector
    vxy = T(vxy.y * clockness,-vxy.x * clockness); // rotate normalized vector orthogonally clockwise as long as
    return vxy;
}
}//namespace


sf::Vector2f B2ToSf::clockwiseNormal(const sf::Vector2f& v1, const sf::Vector2f& v2, float clockness)
{
    // SFML y points down the screen. Clockwise gets turned around. -clockness takes care of that
    return clockwiseNormalImp<sf::Vector2f>(v1, v2, -clockness);
}

b2Vec2 B2ToSf::clockwiseNormal(const b2Vec2& v1, const b2Vec2& v2, float clockness)
{
    return clockwiseNormalImp<b2Vec2>(v1, v2, clockness);
}


sf::Vector2f B2ToSf::clockwiseNormal(const sf::Vector2f& v, float clockness)
{
    return B2ToSf::clockwiseNormal(sf::Vector2f(0,0), v, clockness);
}

b2Vec2 B2ToSf::clockwiseNormal(const b2Vec2& v, float clockness)
{
    return B2ToSf::clockwiseNormal(b2Vec2(0,0), v, clockness);
}

namespace
{
template <typename T>
int rotationClockenssX(const T v11, const T v12, const T v21, const T v22)
{
    T v1 = T(v12.x-v11.x, v12.y-v11.y);
    float length_v1 = std::sqrt(v1.x*v1.x + v1.y*v1.y);

    T v2 = T(v22.x - v21.x, v22.y - v21.y);
    float length_v2 = std::sqrt(v2.x*v2.x + v2.y*v2.y);

    float length_order_missmatch = length_v1 / length_v2;

    if(length_order_missmatch >= 1000 || length_order_missmatch <= 0.0001) // brings lengths of two severely mismatched vectors closer so nothing float-s away...
    {
        v1.x /= length_order_missmatch;
        v1.y /= length_order_missmatch;
    }

    float z = v1.x*v2.y - v2.x*v1.y;

    if(z > 0) return -1; //counter-clockwise
    if(z == 0) return 0; // parallel
    return 1; // clockwise
}
}// namespace


int B2ToSf::rotationClockness(const sf::Vector2f& v11, const sf::Vector2f& v12, const sf::Vector2f& v21, const sf::Vector2f& v22)
{
    // SFML y points down the screen. Clockwise gets turned around. - return takes care of that
    return - rotationClockenssX<sf::Vector2f>(v11, v12, v21, v22);
}

int B2ToSf::rotationClockness(const b2Vec2& v11, const b2Vec2& v12, const b2Vec2& v21, const b2Vec2& v22)
{
    return rotationClockenssX<b2Vec2>(v11, v12, v21, v22);
}


int B2ToSf::rotationClockness(const sf::Vector2f v00, const sf::Vector2f v1, const sf::Vector2f v2)
{
    return B2ToSf::rotationClockness(v00, v1, v00, v2);
}

int B2ToSf::rotationClockness(const b2Vec2 v00, const b2Vec2 v1, const b2Vec2 v2)
{
    return B2ToSf::rotationClockness(v00, v1, v00, v2);
}


int B2ToSf::rotationClockness(const sf::Vector2f v1, const sf::Vector2f v2)
{
    return B2ToSf::rotationClockness(v1, v1, v2);
}

int B2ToSf::rotationClockness(const b2Vec2 v1, const b2Vec2 v2)
{
    return B2ToSf::rotationClockness(b2Vec2(0.f,0.f), v1, v2);
}


sf::Vector2f B2ToSf::extendCorner(const sf::Vector2f& corner, const sf::Vector2f& adjacent1, const sf::Vector2f& adjacent2, float clockness)
{
    sf::Vector2f normal1 = clockwiseNormal(adjacent1, corner, clockness);
    sf::Vector2f normal2 = clockwiseNormal(adjacent2, corner, -clockness);
    sf::Vector2f extension_direction = normal1 + normal2;

    float lengthen_by = clockness*clockness / (normal1.x*extension_direction.x + normal1.y*extension_direction.y);

    extension_direction.x *= lengthen_by;
    extension_direction.y *= lengthen_by;

    return extension_direction;
}
