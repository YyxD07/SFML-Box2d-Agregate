#include "SfFixtureGraphical.h"

B2ToSf::SfFixtureGraphical::SfFixtureGraphical
                   (float outline_thickness, float width, std::size_t point_count,
                    sf::Color fill_color, sf::Color outline_color,
                    sf::Texture texture, sf::Texture outline_texture)
                   :m_outline_thickness(outline_thickness), m_width(width),
                    m_point_count(point_count),
                    m_fill_color(fill_color), m_outline_color(outline_color),
                    m_texture(texture), m_outline_texture(outline_texture)
{}

/*This constructor is specialized for b2_circle */
/*B2ToSf::SfFixtureGraphical::SfFixtureGraphical
                   (float outline_thickness, std::size_t point_count,
                    sf::Color fill_color, sf::Color outline_color,
                    sf::Texture texture, sf::Texture outline_texture)
                   :B2ToSf::SfFixtureGraphical::SfFixtureGraphical(
                    outline_thickness, 1, point_count,
                    fill_color, outline_color,
                    texture, outline_texture)
{}
*/
/*This constructor is specialized for b2_edge and b2_chain */
/*B2ToSf::SfFixtureGraphical::SfFixtureGraphical
                   (float outline_thickness, float width,
                    sf::Color fill_color, sf::Color outline_color,
                    sf::Texture texture, sf::Texture outline_texture)
                   :B2ToSf::SfFixtureGraphical::SfFixtureGraphical(
                    outline_thickness, width, 30,
                    fill_color, outline_color,
                    texture, outline_texture)
{}*/
/*This constructor is specialized for b2_polygon*/
/*B2ToSf::SfFixtureGraphical::SfFixtureGraphical
                   (float outline_thickness,
                    sf::Color fill_color, sf::Color outline_color,
                    sf::Texture texture, sf::Texture outline_texture)
                   :B2ToSf::SfFixtureGraphical::SfFixtureGraphical(
                    outline_thickness, 1,30,
                    fill_color, outline_color,
                    texture, outline_texture)
{}
*/
