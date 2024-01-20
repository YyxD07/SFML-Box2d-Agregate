#include "EdgyFan.h"

B2ToSf::EdgyFan::EdgyFan(std::size_t vertex_count):
    SFShapeBase(sf::PrimitiveType::TriangleFan, vertex_count)
{}

B2ToSf::EdgyFan::EdgyFan(const EdgyFan& edgy_fan_to_copy):
    SFShapeBase(edgy_fan_to_copy)
{std::cout << "EdgyFan copy constructor used\n";}

B2ToSf::EdgyFan::~EdgyFan()
{}


void B2ToSf::EdgyFan::m_makeOutline()
{
    // technically needs range check

    /*rotationClockness needs three points that occupy different positions in space, so first we need to find three such
    points to handle case where edge 0 is the same as edge 1, or for some reason there is another doubled edge*/
    std::size_t prev_edge = 0;
    std::size_t edge = 0;
    for(std::size_t index = prev_edge+1; index < m_vertices.getVertexCount(); ++index)
    {
        if(m_vertices[prev_edge].position != m_vertices[index].position)
        {
            edge = index; // finds first vertex that is not spatially coincidental with vertex 0
            break;
        }
    }
    std::size_t next_edge = edge;
    for(std::size_t index = edge+1; index < m_vertices.getVertexCount(); ++index)
    {
        if(m_vertices[edge].position != m_vertices[index].position)
        {
            next_edge = index;
            break;
        }
    }


    /*Determines rotation of edge from m_outline_vertices[0] to m_outline_vertices[0] in relation to edge from m_outline_vertices[0]
    to m_outline_vertices[2] is clockwise (1) or counter clockwise(-1).*/
    float fan_clockness = B2ToSf::rotationClockness(m_vertices[edge].position, m_vertices[prev_edge].position,
                                                    m_vertices[next_edge].position);

    /*Fan_clockness is now either 1 or -1. This number multiplied with m_outline_thickness will make sure that edges extend far enough
    out so that edge stripe will have specified outline thickness.*/
    fan_clockness *= m_outline_thickness;


    //Deal with edges from 2 to end-1
    for(std::size_t index = 2; index < m_vertices.getVertexCount()-1; ++index)
    {
        m_extendCorner(index,index-1,index+1,fan_clockness);
    }

    //Check if fan of triangles if fully opened, that is vertex 1 and last vertex are in same position
    if(m_vertices[1].position != m_vertices[m_vertices.getVertexCount()-1].position)
    {/*When they are not in the same position  there is a triangle "missing" and vertex 0 gets included in outline of the
     fan.*/
    m_extendCorner(0,m_vertices.getVertexCount()-1, 1, fan_clockness);
    m_extendCorner(1,0,2,fan_clockness);
    m_extendCorner(m_vertices.getVertexCount()-1,m_vertices.getVertexCount()-2,0,fan_clockness);
    }
    else
    {/*When fan is fully opened, and there is no "missing" triangle.*/
    m_extendCorner(1,m_vertices.getVertexCount()-2,2,fan_clockness);
    //m_outline_vertices[0].position = m_vertices[0].position;
    m_outline_vertices[0].position = m_outline_vertices[1].position;
    m_outline_vertices[m_outline_vertices.getVertexCount()-1].position = m_outline_vertices[1].position;

    }











}
