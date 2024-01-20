#include "EdgyStripe.h"

B2ToSf::EdgyStripe::EdgyStripe(std::size_t vertex_count):
    SFShapeBase(sf::PrimitiveType::TriangleStrip, vertex_count)
{}

B2ToSf::EdgyStripe::EdgyStripe(const EdgyStripe& edgy_stripe_to_copy):
    SFShapeBase(edgy_stripe_to_copy)
{std::cout << "EdgyStripe copy constructor used\n";}

B2ToSf::EdgyStripe::~EdgyStripe()
{}



void B2ToSf::EdgyStripe::m_makeOutline()
{
    // technically needs range check
    /*Determines rotation of edge from m_outline_vertices[0] to m_outline_vertices[2] in relation to edge from m_outline_vertices[0]
    to m_outline_vertices[1] is clockwise (1) or counter clockwise(-1). Two edges should not be collinear!*/
    float stripe_clockness = B2ToSf::rotationClockness(m_vertices[0].position, m_vertices[1].position,m_vertices[2].position);

    /*Stripe_clockness is now either 1 or -1. This number multiplied with m_outlineJ_thickness will make sure that edges extend far enough
    out so that edge stripe will have specified outline thickness.*/
    stripe_clockness *= m_outline_thickness;


    /*First vertex of triangle stripe is forms outside edges with vertices 1 and 2*/
    m_extendCorner(0,1,2,stripe_clockness);

    /*Second vertex forms one outside edge with vertex 0, the second outside edge is formed with edge 3*/
    m_extendCorner(1,0,3,-stripe_clockness);

    /*Vertices in range from third to third to last form two outside edges with vertices that precede them by 2 and
    one that follow them by 2. How they are dealt with depends on if they are odd or even.*/
    int odd_even = 1;//odd_even will change how vertex is dealt with according to it being odd or even
    std::size_t index = 2;
    for(;index < m_vertices.getVertexCount()-2;odd_even *= -1, ++index)//Changes odd_even between +1 and -1, for odd and even vertices
    {
        m_extendCorner(index,index-2,index+2,stripe_clockness*odd_even);
    }

    /*Second to last vertex forms one edge with vertex preceding it by 2, while the second outside edge is formed
    with vertex that follows it by 1*/
    m_extendCorner(index, index-2, index+1, stripe_clockness*odd_even);
    ++index;
    odd_even *= -1;

    /*Last vertex forms one outside edge with vertex preceding it by two and one outside edge by vertex preceding it by 1*/
    m_extendCorner(index,index-2,index-1,stripe_clockness*odd_even);

    /*This code should take care of and edge formed if last two vertices match the first two... */
    if(m_vertices[0].position == m_vertices[m_vertices.getVertexCount()-2].position &&
       m_vertices[1].position == m_vertices[m_vertices.getVertexCount()-1].position)
    {
        m_extendCorner(0,2,m_vertices.getVertexCount()-3, -stripe_clockness);
        m_outline_vertices[m_vertices.getVertexCount()-2].position = m_outline_vertices[0].position;

        m_extendCorner(1,3,m_vertices.getVertexCount()-4, stripe_clockness);
        m_outline_vertices[m_vertices.getVertexCount()-1].position = m_outline_vertices[1].position;
    }

}




































