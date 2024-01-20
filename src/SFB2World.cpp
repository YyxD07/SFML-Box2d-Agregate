#include "SFB2World.h"

B2ToSf::SFB2World::SFB2World()
{
    //ctor
}

B2ToSf::SFB2World::~SFB2World()
{
    //dtor
}

void B2ToSf::SFB2World::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    for(auto& cluster : m_cluster_agregate)
    {
        cluster->draw(target, states);
    }
}

/*B2ToSf::SFB2Cluster& B2ToSf::SFB2World::addBody(std::unique_ptr<B2ToSf::SFB2Cluster> && cluster)
{
    m_body_cluster.emplace_back(std::move(body));
    return *(m_body_cluster.back());
}*/
