#ifndef SFSHAPEBASE_H
#define SFSHAPEBASE_H

#include "Graphics.hpp"
#include "box2d.h"

#include "ConstantSettings.h"

#include "clockwiseNormal.h"

#include <vector>

#include <iostream>

namespace B2ToSf
{

class SFShapeBase: public sf::Drawable, public sf::Transformable
{
    public:
        SFShapeBase(sf::PrimitiveType type = sf::PrimitiveType::Points, std::size_t point_count = 0);
        SFShapeBase(const SFShapeBase& base_to_copy);
        virtual ~SFShapeBase();

        virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const; //Used by sf::RenderWinodw to draw object
        virtual sf::Vector2f getPoint(std::size_t index) const; //Returns vertex with index
        virtual std::size_t getPointCount() const; //Returns number of vertices
        void setPoint(std::size_t index, const sf::Vector2f& point); // Set vertex with index
        void setPoints(const sf::VertexArray& new_vertices); // Set all vertices
        void setPointCount(std::size_t count); // Set number of vertices
        void setFillColor(const sf::Color& color);
        void setOutlineThickness(float outline_thickness);
        void setOutlineColor(const sf::Color& color);

    protected:

        /*Wraps extend corner. Member function takes only vertex indices and does all changes to member vertices directly. This should
        make code of m_makeOutline() a bit less verbose... */
        void m_extendCorner(std::size_t index0, std::size_t index1, std::size_t index2, float clockness);
        virtual void m_makeOutline(); // Needs at least 4 vertices

        float m_outline_thickness;


        sf::Color m_fill_color;
        sf::Color m_outline_color;

        sf::VertexArray m_vertices;
        sf::VertexArray m_outline_vertices;

        sf::Texture m_texture;
        sf::Texture m_outline_texture; // Always empty

        sf::VertexArray * m_pt_top_vertices;
        sf::VertexArray * m_pt_bottom_vertices;
};

}//namespace B2ToSf
#endif // SFSHAPEBASE_H
