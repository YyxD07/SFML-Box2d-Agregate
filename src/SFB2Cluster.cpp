#include "SFB2Cluster.h"


B2ToSf::SFB2Cluster::SFB2Cluster(){}
B2ToSf::SFB2Cluster::~SFB2Cluster(){}


void B2ToSf::SFB2Cluster::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    for(auto& body : m_body_cluster)
    {
        body->draw(target, states);
    }
}

void B2ToSf::SFB2Cluster::addBody(std::unique_ptr<B2ToSf::SFB2Body> && body)
{
    m_body_cluster.emplace_back(std::move(body));
}

void B2ToSf::SFB2Cluster::destroyBody(const std::unique_ptr<B2ToSf::SFB2Body> & doomed_body)
{
    for(auto it = m_body_cluster.begin(); it != m_body_cluster.end(); ++it)
    {
        if(*it == doomed_body)
        {
            //First destroy box2d body and only then destroy SFB2Body!
            (*it)->getBody()->GetWorld()->DestroyBody((*it)->getBody());
            m_body_cluster.erase(it);
            break; // no need to deal with any other elements of the cluster
        }
    }
}

void B2ToSf::SFB2Cluster::reacquaintSFB2Fixts()
{
    for(auto & body : m_body_cluster)
    {
        body->reacquaintSfB2Fixts();
    }
}

std::vector<std::unique_ptr<B2ToSf::SFB2Body>> & B2ToSf::SFB2Cluster::getBodyCluster()
{
    return m_body_cluster;
}

