#ifndef SFB2WORLD_H
#define SFB2WORLD_H

#include <memory>
#include <vector>

//#include "Graphics.hpp"
#include "box2d.h"

#include "SFB2Cluster.h"

namespace B2ToSf
{
class SFB2World: public sf::Drawable
{
    public:
        SFB2World(std::size_t size_of_cluster_agregate);
        virtual ~SFB2World();

        virtual void draw(sf::RenderTarget&, sf::RenderStates) const; //Used by sf::RenderWinodw to draw all clusters

        B2ToSf::SFB2Cluster & addCluster(std::unique_ptr<B2ToSf::SFB2Cluster> && cluster);

        void destroyCluster(const std::unique_ptr<B2ToSf::SFB2Cluster> & doomed_cluster);

        void reacquaintSFB2Fixts(); //Reacquaints bodies of the whole cluster

        std::vector<std::unique_ptr<B2ToSf::SFB2Cluster>> & getClusterAgregate();

    protected:

    std::vector<std::unique_ptr<B2ToSf::SFB2Cluster>> m_cluster_agregate;

    private:
};
}// namesapce B2ToSf

#endif // SFB2WORLD_H
