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
        using fixt_pair_vector = std::vector<fixt_pair>;
        using fixt_iter = fixt_pair_vector::iterator;
        using fixt_rev_iter = fixt_pair_vector::reverse_iterator;

        SFB2Body(b2Body* body);
        virtual ~SFB2Body();

        //Ads a single fixture to a body:
        void addFixture(const b2FixtureDef* fixt_def,
                        const SfFixtureGraphical& graphical = SfFixtureGraphical());
        void addFixture(const b2Shape* b2_shape, const float densitiy = 1,
                        const SfFixtureGraphical& graphical = SfFixtureGraphical());

        //Destroys single fixture
        void destroyFixture(const fixt_iter & target_pair);
        void destroyFixture(const fixt_rev_iter & target_pair);
        void destroyFixture(const b2Fixture* b2_fixture);
        void destroyFixture(const std::unique_ptr<B2ToSf::SFShapeBase> & sf_fixture);

        //Sets position and rotation of SFML fixture representations in accordance with box2d fixtures:
        //Should be called before calling sfml draw on the body.
        void reacquaintSfB2Fixts();


        b2Body* getBody();
        fixt_pair_vector& getFixtVec();




    protected:
    b2Body* m_body;
    // Vector of SFML representations of Box2d fixtures that belong to body
    fixt_pair_vector m_fixutres;




    private:
        virtual void draw(sf::RenderTarget&, sf::RenderStates) const; //Used by sf::RenderWinodw to draw object



};
}//namesapce B2ToSf

#endif // SFMLB2D_H
