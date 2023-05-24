#include "SFB2Body.h"


B2ToSf::SFB2Body::SFB2Body(b2Body* body): m_body(body){}

B2ToSf::SFB2Body::~SFB2Body(){}

void B2ToSf::SFB2Body::addFixture(const b2FixtureDef* fixt_def, const SfFixtureGraphical& graphical)
{
std::unique_ptr<B2ToSf::SFShapeBase> temp_shape_ptr;
switch(fixt_def->shape->GetType())
{
    case 0: //circle
        temp_shape_ptr = Transl8::b2ToSFMLCircle(*(fixt_def->shape), graphical.m_point_count);
        //m_fixutres.emplace_back(std::make_pair(Transl8::b2ToSFMLCircle(*(fixt_def->shape)), nullptr));
        break;
    case 1: //edge
        temp_shape_ptr = Transl8::b2ToSFMLEdge(*(fixt_def->shape), graphical.m_width);
        //m_fixutres.emplace_back(std::make_pair(Transl8::b2ToSFMLEdge(*(fixt_def->shape)), nullptr));
        break;
    case 2: //polygon
        temp_shape_ptr = Transl8::b2ToSFMLPolygon(*(fixt_def->shape));
        //m_fixutres.emplace_back(std::make_pair(Transl8::b2ToSFMLCircle(*(fixt_def->shape)), nullptr));
        break;
    case 3: //chain
        temp_shape_ptr = Transl8::b2ToSFMLChain(*(fixt_def->shape), graphical.m_width);
        //m_fixutres.emplace_back(std::make_pair(Transl8::b2ToSFMLChain(*(fixt_def->shape)), nullptr));
        break;
    default://4 typeCount
        break;
}



//B2ToSf::SFShapeBase.setOutlineColor();
temp_shape_ptr->setFillColor(graphical.m_fill_color);
temp_shape_ptr->setOutlineThickness(graphical.m_outline_thickness);
temp_shape_ptr->setOutlineColor(graphical.m_outline_color);

b2Fixture* temp_fixt = m_body->CreateFixture(fixt_def);
temp_fixt->GetUserData().pointer = reinterpret_cast<uintptr_t>(this); //This this needs more thinking done...

m_fixutres.emplace_back(std::make_pair(std::move(temp_shape_ptr), std::move(temp_fixt)));
}


void B2ToSf::SFB2Body::addFixture(const b2Shape* b2_shape, const float densitiy, const SfFixtureGraphical& graphical)
{
    b2FixtureDef temp_fixt_def;
    temp_fixt_def.shape = b2_shape;
    temp_fixt_def.density = densitiy;

    addFixture(&temp_fixt_def, graphical);
}


void B2ToSf::SFB2Body::destroyFixture(const fixt_iter & target_pair)
{
    m_body->DestroyFixture(target_pair->second);
    m_fixutres.erase(target_pair);
}

void B2ToSf::SFB2Body::destroyFixture(const b2Fixture* b2_fixture)
{
    for(auto it = m_fixutres.begin(); it != m_fixutres.end();)
    {
        if(it->second == b2_fixture)
        {
            destroyFixture(it);
            break;
        }

    }
}

void B2ToSf::SFB2Body::destroyFixture(const std::unique_ptr<B2ToSf::SFShapeBase> & sf_fixture)
{
    for(auto it = m_fixutres.begin(); it != m_fixutres.end();)
    {
        if(it->first == sf_fixture)
        {
            destroyFixture(it);
            break;
        }
    }
}

void B2ToSf::SFB2Body::reacquaintSfB2Fixts()
{
    sf::Vector2f position = Transl8::vec2(m_body->GetPosition());
    float angle = Transl8::getRotation(m_body);

    for(auto& fixt_pair : m_fixutres)
    {
        fixt_pair.first->setPosition(position);
        fixt_pair.first->setRotation(angle);
    }

}



b2Body* B2ToSf::SFB2Body::getBody()
{
    return m_body;
}

std::vector<B2ToSf::SFB2Body::fixt_pair>& B2ToSf::SFB2Body::getFixtVec()
{
    return m_fixutres;
}

void B2ToSf::SFB2Body::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    for(auto& duo : m_fixutres)
    {
        duo.first->draw(target, states);
    }
}





























