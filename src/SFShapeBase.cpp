#include "SFShapeBase.h"

B2ToSf::SFShapeBase::SFShapeBase(sf::PrimitiveType type, std::size_t point_count):
    m_outline_thickness(0.0f),
    m_fill_color(sf::Color::White),
    m_outline_color(sf::Color::White),
    m_vertices(type, point_count),
    m_outline_vertices(type,point_count),
    m_texture(),
    m_outline_texture(),
    m_pt_top_vertices(&m_vertices),
    m_pt_bottom_vertices(&m_outline_vertices)
{}

B2ToSf::SFShapeBase::SFShapeBase(const SFShapeBase& base_to_copy):
    m_outline_thickness(base_to_copy.m_outline_thickness),
    m_fill_color(base_to_copy.m_fill_color),
    m_outline_color(base_to_copy.m_outline_color),
    m_vertices(base_to_copy.m_vertices),
    m_outline_vertices(base_to_copy.m_outline_vertices),
    m_texture(),
    m_outline_texture(),
    m_pt_top_vertices(&m_vertices),
    m_pt_bottom_vertices(&m_outline_vertices)
{std::cout << "SFShapeBase copy constructor used\n";}


B2ToSf::SFShapeBase::~SFShapeBase()
{}





void B2ToSf::SFShapeBase::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    if(m_outline_thickness != 0)
    {
        states.transform *= getTransform();
        states.texture = &m_outline_texture;
        target.draw(*m_pt_bottom_vertices, states);

        states.texture = &m_texture;
        target.draw(*m_pt_top_vertices, states);

    }
    else
    {
        states.transform *= getTransform();

        states.texture = &m_texture;

        target.draw(m_vertices, states);

    }

}

sf::Vector2f B2ToSf::SFShapeBase::getPoint(std::size_t index) const
{
    // out of range check missing
    return m_vertices[index].position;
}

std::size_t B2ToSf::SFShapeBase::getPointCount() const
{
    return m_vertices.getVertexCount();
}

void B2ToSf::SFShapeBase::setPoint(std::size_t index, const sf::Vector2f& point)
{
    // out of range check
    m_vertices[index].position = point;
}

void B2ToSf::SFShapeBase::setPoints(const sf::VertexArray& new_vertices)
{

    setPointCount(new_vertices.getVertexCount());

    for(std::size_t index = 0; index < m_vertices.getVertexCount(); ++index)
    {
        B2ToSf::SFShapeBase::setPoint(index, new_vertices[index].position);
    }
}

void B2ToSf::SFShapeBase::setPointCount(std::size_t count)
{
    m_vertices.resize(count);
    m_outline_vertices.resize(count);
}

void B2ToSf::SFShapeBase::setFillColor(const sf::Color& color)
{
    m_fill_color = color;

    for(std::size_t index = 0; index < m_vertices.getVertexCount(); ++index)
    {
        (*m_pt_top_vertices)[index].color = m_fill_color;
    }
}

void B2ToSf::SFShapeBase::setOutlineThickness(float outline_thickness)
{
    m_outline_thickness = outline_thickness;
    m_makeOutline();

    if(m_outline_thickness >= 0)
    {
        m_pt_top_vertices = &m_vertices;
        m_pt_bottom_vertices = &m_outline_vertices;
    }
    else
    {
        m_pt_top_vertices = &m_outline_vertices;
        m_pt_bottom_vertices = &m_vertices;
    }
    setFillColor(m_fill_color);
    setOutlineColor(m_outline_color);

}

void B2ToSf::SFShapeBase::setOutlineColor(const sf::Color& color)
{
    m_outline_color = color;

    for(std::size_t index = 0; index < m_outline_vertices.getVertexCount(); ++index)
    {
        (*m_pt_bottom_vertices)[index].color = m_outline_color;
    }
}

void B2ToSf::SFShapeBase::m_extendCorner(std::size_t index0, std::size_t index1, std::size_t index2, float clockness)
{
    m_outline_vertices[index0].position = extendCorner(m_vertices[index0].position,
                                                       m_vertices[index1].position,
                                                       m_vertices[index2].position,
                                                       clockness);
    m_outline_vertices[index0].position.x += m_vertices[index0].position.x;
    m_outline_vertices[index0].position.y += m_vertices[index0].position.y;
}

void B2ToSf::SFShapeBase::m_makeOutline()
{
    for(std::size_t index = 0; index < m_outline_vertices.getVertexCount(); ++ index )
    {
        m_outline_vertices[index].position = m_vertices[index].position;
    }
}


