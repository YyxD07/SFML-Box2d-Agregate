#ifndef SFFIXTUREGRAPHICAL_H
#define SFFIXTUREGRAPHICAL_H

#include "Graphics.hpp"

namespace B2ToSf
{
struct SfFixtureGraphical
{
public:

/*This constructor is the general one, meant to set graphical properties for all SFML representations of Box2d fixtures.
This means that some properties are redundant in some cases. m_width is used only for b2_edge and point_count is used only
for b2_circle*/
SfFixtureGraphical(float outline_thickness = 0, float width = 1, std::size_t point_count = 30,
                    sf::Color fill_color = sf::Color::White, sf::Color outline_color = sf::Color::Red,
                    sf::Texture texture = sf::Texture(), sf::Texture outline_texture = sf::Texture());

/*This constructor is specialized for b2_circle */
/*SfFixtureGraphical(float outline_thickness = 0, std::size_t point_count = 30,
                    sf::Color fill_color = sf::Color::White, sf::Color outline_color = sf::Color::Red,
                    sf::Texture texture = sf::Texture(), sf::Texture outline_texture = sf::Texture());
*/
/*This constructor is specialized for b2_edge and b2_chain */
/*SfFixtureGraphical(float outline_thickness = 0, float width = 1,
                    sf::Color fill_color = sf::Color::White, sf::Color outline_color = sf::Color::Red,
                    sf::Texture texture = sf::Texture(), sf::Texture outline_texture = sf::Texture());
*/
/*This constructor is specialized for b2_polygon*/
/*SfFixtureGraphical(float outline_thickness = 0,
                    sf::Color fill_color = sf::Color::White, sf::Color outline_color = sf::Color::Red,
                    sf::Texture texture = sf::Texture(), sf::Texture outline_texture = sf::Texture());
*/


float m_outline_thickness;
float m_width;

std::size_t m_point_count;

sf::Color m_fill_color;
sf::Color m_outline_color;

sf::Texture m_texture;
sf::Texture m_outline_texture;

};
}// namespace B2ToSf

#endif // SFFIXTUREGRAPHICAL_H
