#include "Transl8.h"


sf::Vector2f Transl8::vec2(const b2Vec2& xy)
{
    return sf::Vector2f(xy.x*UniversalConstants::kScaling,-xy.y*UniversalConstants::kScaling);
}

// untested!
b2Vec2 Transl8::vec2(const sf::Vector2f& xy)
{
    return b2Vec2(xy.x/UniversalConstants::kScaling, -xy.y/UniversalConstants::kScaling);
}

float Transl8::getRotation(const sf::Transformable& object)
{
    return -(object.getRotation()/360) * 2 * UniversalConstants::kPi;
}


float Transl8::getRotation(b2Body* object)
{
    return -(object->GetAngle())/(2*UniversalConstants::kPi)*360;
}

B2ToSf::EdgyFan Transl8::convexPolygon(const b2PolygonShape& b2_polygon)
{
    B2ToSf::EdgyFan edgy_fan(b2_polygon.m_count + 2);

    //Determing min an max x and y - AABB
    float minx = b2_polygon.m_vertices[0].x;
    float maxx = b2_polygon.m_vertices[0].x;
    float miny = b2_polygon.m_vertices[0].y;
    float maxy = b2_polygon.m_vertices[0].y;
    for(std::size_t index = 0; index < static_cast<std::size_t>(b2_polygon.m_count); ++index)
    {
        if(b2_polygon.m_vertices[index].x < minx){minx = b2_polygon.m_vertices[index].x;}
        else if(b2_polygon.m_vertices[index].x > maxx){maxx = b2_polygon.m_vertices[index].x;}
        if(b2_polygon.m_vertices[index].y < miny){miny = b2_polygon.m_vertices[index].y;}
        else if(b2_polygon.m_vertices[index].y > maxy){maxy = b2_polygon.m_vertices[index].y;}
    }

    /*Center of AABB of convex polygon should always lie inside or on one of the edges of that
    convex polygon.*/
    float midx = (minx + maxx)/2;
    float midy = (miny + maxy)/2;
    // Set vertex 0 to center of AABB to ensure that it is contained inside of EdgyFan
    edgy_fan.setPoint(0,Transl8::vec2(b2Vec2(midx, midy)));

    /*This copies b2_poligon vertices to sfml EdgyFan.*/
    for(std::size_t index = 1; index < static_cast<std::size_t>(b2_polygon.m_count) + 1; ++index)
    {
        edgy_fan.setPoint(index, Transl8::vec2(b2_polygon.m_vertices[index - 1]));
    }
    edgy_fan.setPoint(b2_polygon.m_count + 1, edgy_fan.getPoint(1));

    edgy_fan.setPosition(Transl8::vec2(b2_polygon.m_centroid));
    return edgy_fan;


}


B2ToSf::EdgyFan Transl8::line (const b2EdgeShape& b2_edge, float width)
/***Will need to deal with v0 and v4 of one sided edge some time in the future?? ***/
{
    b2Vec2 moved_to_oo = b2_edge.m_vertex2 - b2_edge.m_vertex1;
    moved_to_oo.Normalize();

    width /= UniversalConstants::kScaling;
    b2Vec2 normal(moved_to_oo.y * width/2,-moved_to_oo.x * width/2); // Vector orthogonal to the edge, half width long.

    //Should create 4 vertices offset half the width around both sides of the edge
    b2Vec2 edgeOffsetVertices[4] {
        b2Vec2(b2_edge.m_vertex1.x+normal.x,b2_edge.m_vertex1.y+normal.y),
        b2Vec2(b2_edge.m_vertex1.x-normal.x,b2_edge.m_vertex1.y-normal.y),
        b2Vec2(b2_edge.m_vertex2.x+normal.x,b2_edge.m_vertex2.y+normal.y),
        b2Vec2(b2_edge.m_vertex2.x-normal.x,b2_edge.m_vertex2.y-normal.y)};

    b2PolygonShape fluffed_up_edge;
    fluffed_up_edge.Set(edgeOffsetVertices,4);
    return Transl8::convexPolygon(fluffed_up_edge);
}

/***Takes box2d circle and returns sfml circle... ***/
B2ToSf::EdgyFan Transl8::circle(const b2CircleShape& b2_circle, std::size_t point_count)
{
    B2ToSf::EdgyFan circle_fan(point_count + 2);
    circle_fan.setPoint(0,sf::Vector2f(0.f,0.f));
    b2Vec2 vertex_in_b2;
    double angle_step = UniversalConstants::kPi*2/point_count;
    for(std::size_t step = 0; step < point_count; ++step)
    {
        vertex_in_b2.x = std::sin(angle_step*step)*b2_circle.m_radius;
        vertex_in_b2.y = std::cos(angle_step*step)*b2_circle.m_radius;
        circle_fan.setPoint(step+1,Transl8::vec2(vertex_in_b2));
    }

    circle_fan.setPoint(point_count+1,circle_fan.getPoint(1));

    return circle_fan;
}

B2ToSf::EdgyStripe Transl8::chainShape(const b2ChainShape& b2_chain, float width)
{
    if(width == 0) width = 0.0001; // When width is 0 there also can be no outline

    int32 b2_count{b2_chain.m_count};
    bool chain_forms_loop = (b2_chain.m_prevVertex == b2_chain.m_vertices[b2_count]) &&
                            (b2_chain.m_nextVertex == b2_chain.m_vertices[1]);


    std::size_t sf_count{static_cast<std::size_t>(2*b2_count)};
    if(chain_forms_loop) sf_count += 2; //If chain forms  a loop we need two additional vertices to close the loop

    B2ToSf::EdgyStripe sf_chain(sf_count);

    for(int32 index = 0; index < b2_count; ++index)
    {
        /*First surface vertices are set, that is every second vertex. If chain is a loop the last surface vertex will need
        to be added. */
        sf_chain.setPoint(index*2, Transl8::vec2(b2_chain.m_vertices[index]));
    }

    /*First vertex for the width of the chain (index = 1) has the direction determined by b2_chain.m_prevVertex and vertex 1*/
    sf_chain.setPoint(1, sf_chain.getPoint(0) +
                      B2ToSf::extendCorner(sf_chain.getPoint(0),
                                           Transl8::vec2(b2_chain.m_prevVertex),
                                           sf_chain.getPoint(2),-width));

    for(std::size_t index = 2; index < static_cast<std::size_t>(b2_count*2); index += 2)
    {
        /*Second width vertices can be calculated from surface vertices. If chain is a loop last width vertex will need to b
        added*/
        sf_chain.setPoint(index+1, sf_chain.getPoint(index) +
                          B2ToSf::extendCorner(sf_chain.getPoint(index),
                                               sf_chain.getPoint(index-2),
                                               sf_chain.getPoint(index+2), -width));
    }

    sf_chain.setPoint(b2_count*2 - 1, sf_chain.getPoint(b2_count*2 - 2) +
                      B2ToSf::extendCorner(sf_chain.getPoint(b2_count*2-2),
                                           sf_chain.getPoint(b2_count*2-4),
                                           Transl8::vec2(b2_chain.m_nextVertex),-width));



    if(chain_forms_loop)
    {
        /*If chain form a loop last vertex must be added to close sfml triangle stripe loop. */

        sf_chain.setPoint(sf_count - 2, sf_chain.getPoint(0));
        sf_chain.setPoint(sf_count - 1, sf_chain.getPoint(1));

    }


    return sf_chain;
}



/*There is no reason for these to be wrapper funcitons, they really should do all the work of wrapped functions but
should work with new objects made on heap from the start...*/

std::unique_ptr<B2ToSf::EdgyFan> Transl8::b2ToSFMLPolygon(const b2Shape& b2_polygon)
{
    //const b2PolygonShape& neki = dynamic_cast<const b2PolygonShape&>(b2_polygon);
    return std::make_unique<B2ToSf::EdgyFan>(Transl8::convexPolygon(dynamic_cast<const b2PolygonShape&>(b2_polygon)));
}
std::unique_ptr<B2ToSf::EdgyFan> Transl8::b2ToSFMLEdge(const b2Shape& b2_edge, float width)
{
    return std::make_unique<B2ToSf::EdgyFan>(Transl8::line(dynamic_cast<const b2EdgeShape&>(b2_edge), width));
}
std::unique_ptr<B2ToSf::EdgyFan> Transl8::b2ToSFMLCircle(const b2Shape& b2_circle, std::size_t point_count)
{
    return std::make_unique<B2ToSf::EdgyFan>(Transl8::circle(dynamic_cast<const b2CircleShape&>(b2_circle),point_count));
}
std::unique_ptr<B2ToSf::EdgyStripe> Transl8::b2ToSFMLChain(const b2Shape& b2_chain, float width)
{
    return std::make_unique<B2ToSf::EdgyStripe>(Transl8::chainShape(dynamic_cast<const b2ChainShape&>(b2_chain),width));
}

















