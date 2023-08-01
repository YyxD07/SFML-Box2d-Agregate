#ifndef SFB2CLUSTER_H
#define SFB2CLUSTER_H

#include <memory>
#include <vector>

#include "Graphics.hpp"
#include "box2d.h"

#include "SFB2Body.h"

namespace B2ToSf
{

/**
This is only a container for holding bodies. It does not create them. It can hold bodies belonging to different
b2Worlds. There is probably no reason to mix different b2Worlds, but you could do it in this container.

Member function draw, draws all the bodies held in a batch. This means that clusters can be used to manage sequence of
drawing to screen of whole batches of bodies.**/

class SFB2Cluster: public sf::Drawable
{
    public:
        SFB2Cluster(std::size_t size_of_cluster = 0);
        virtual ~SFB2Cluster();

        virtual void draw(sf::RenderTarget&, sf::RenderStates) const; //Used by sf::RenderWinodw to draw object clusters

        /*Moves body created somewhere else to this cluster. Return reference to added body can be used to add fixtures to
        newly created body.*/
        B2ToSf::SFB2Body& addBody(std::unique_ptr<B2ToSf::SFB2Body> && body);

        /*Destroys body, its fixtures and corresponding box2d entities.*/
        void destroyBody(const std::unique_ptr<B2ToSf::SFB2Body> & doomed_body);

        void reacquaintSFB2Fixts(); //Reacquaints bodies of the whole cluster

        std::vector<std::unique_ptr<B2ToSf::SFB2Body>> & getBodyCluster();


    protected:
        std::vector<std::unique_ptr<B2ToSf::SFB2Body>> m_body_cluster;

    private:




};

}// namespace B2ToSf
#endif // SFB2CLUSTER_H
