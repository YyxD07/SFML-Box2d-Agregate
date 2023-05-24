#ifndef SFB2BODY_H
#define SFB2BODY_H

#include <memory>
#include <vector>

#include "box2d.h"
#include "SFShapeBase.h"
#include "EdgyFan.h"
#include "EdgyStripe.h"
#include "SfFixtureGraphical.h"
#include "Transl8.h"



namespace B2ToSf
{

class SFB2Body: public sf::Drawable
{
    public:

        using fixt_pair = std::pair<std::unique_ptr<B2ToSf::SFShapeBase>, b2Fixture*>;
        using fixt_iter = std::vector<fixt_pair>::iterator;

        SFB2Body(b2Body* body);
        virtual ~SFB2Body();

        void addFixture(const b2FixtureDef* fixt_def,
                        const SfFixtureGraphical& graphical = SfFixtureGraphical());
        void addFixture(const b2Shape* b2_shape, const float densitiy = 1,
                        const SfFixtureGraphical& graphical = SfFixtureGraphical());

        void destroyFixture(const fixt_iter & target_pair);
        void destroyFixture(const b2Fixture* b2_fixture);
        void destroyFixture(const std::unique_ptr<B2ToSf::SFShapeBase> & sf_fixture);

        void reacquaintSfB2Fixts();


        b2Body* getBody();
        std::vector<fixt_pair>& getFixtVec();






    protected:
    b2Body* m_body;
    // Vector of SFML representations of Box2d fixtures that belong to body
    std::vector<std::pair<std::unique_ptr<B2ToSf::SFShapeBase>, b2Fixture*>> m_fixutres;




    private:
        virtual void draw(sf::RenderTarget&, sf::RenderStates) const; //Used by sf::RenderWinodw to draw object



};
}//namesapce B2ToSf

#endif // SFMLB2D_H
