#include "SFB2World.h"

B2ToSf::SFB2World::SFB2World(std::size_t size_of_cluster_agregate)
{
    m_cluster_agregate.reserve(size_of_cluster_agregate);
}

B2ToSf::SFB2World::~SFB2World(){}

void B2ToSf::SFB2World::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    for(auto& cluster : m_cluster_agregate)
    {
        cluster->draw(target, states);
    }
}

B2ToSf::SFB2Cluster& B2ToSf::SFB2World::addCluster(std::unique_ptr<B2ToSf::SFB2Cluster> && cluster)
{
     m_cluster_agregate.emplace_back(std::move(cluster));
     return *(m_cluster_agregate.back());
}


void B2ToSf::SFB2World::destroyCluster(const std::unique_ptr<B2ToSf::SFB2Cluster> & doomed_cluster)
{
    for(auto it =m_cluster_agregate.begin(); it != m_cluster_agregate.end(); ++it)
    {
        if(*it == doomed_cluster)
        {
            m_cluster_agregate.erase(it);
            break;
        }
    }
}


void B2ToSf::SFB2World::reacquaintSFB2Fixts()
{
    for(auto & cluster : m_cluster_agregate)
    {
        cluster->reacquaintSFB2Fixts();
    }
}

std::vector<std::unique_ptr<B2ToSf::SFB2Cluster>> & B2ToSf::SFB2World::getClusterAgregate()
{
    return m_cluster_agregate;
}
