#include <iostream>
#include <SFML/Window.hpp>
#include <Graphics.hpp>
#include "box2d.h"
//#include "box2d-main/include/box2d/box2d.h"

#include "ConstantSettings.h"

#include "Transl8.h"

#include "EdgyStripe.h"

#include "EdgyFan.h"

#include "SFB2Body.h"



int main()
{

     //world with gravity. World holds and handles all in box2d
    b2Vec2 gravity(0.0f, -5.0f);
    b2World world(gravity);
        // window is kind of world-ish?
        sf::RenderWindow worldViable(sf::VideoMode(UniversalConstants::kWindowWidth,UniversalConstants::kWindowHeight),"?");

    // ground box executed with static body
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f,-150.0f);
    b2Body* groundBody = world.CreateBody(&groundBodyDef); //world is the one to create and destroy bodies


    b2PolygonShape groundBox;
    groundBox.SetAsBox(150.0f,10.0f); //this creates box 100 units wide and 20 units high. 2 times the value of function parameters.

    groundBody->CreateFixture(&groundBox,0.0f);

    groundBody->SetTransform(groundBody->GetPosition(),0.1);
        //sf reprensetation Origin in top left corner, b2_x = sfml_x, b2y = -sfml_y


        B2ToSf::EdgyFan groundBodyVisable = Transl8::convexPolygon(groundBox);
        groundBodyVisable.setPosition(Transl8::vec2(groundBody->GetPosition()));
        groundBodyVisable.setRotation(Transl8::getRotation(groundBody));
        groundBodyVisable.setFillColor(sf::Color::Red);
        groundBodyVisable.setOutlineColor(sf::Color::Green);
        groundBodyVisable.setOutlineThickness(4);

    /**First test of SFML Box2d agregate**/
    b2BodyDef agregateBallDef;
    agregateBallDef.type = b2_dynamicBody;
    agregateBallDef.position.Set(0.f,0.f);

    B2ToSf::SFB2Body agregateBall(world.CreateBody(&agregateBallDef));

    b2CircleShape agregateBallShape;
    agregateBallShape.m_radius = 2;
    b2FixtureDef agregateBallFixtDef;
    agregateBallFixtDef.shape = &agregateBallShape;
    agregateBallFixtDef.restitution = 0.1f;
    agregateBallFixtDef.friction = 0.3;
    agregateBallFixtDef.density = 2;

    b2FixtureDef* agregateBallFixtDefPtr = &agregateBallFixtDef;

    agregateBall.addFixture(agregateBallFixtDefPtr);

    b2PolygonShape agregateBoxShape;
    agregateBoxShape.SetAsBox(1.f,3.f);
    b2FixtureDef agregateBoxFixtDef;
    agregateBoxFixtDef.shape = &agregateBoxShape;
    agregateBoxFixtDef.restitution = 0.1;
    agregateBoxFixtDef.friction = 0.5;
    agregateBoxFixtDef.density = 3;

    b2FixtureDef* agregateBoxFixtDefPtr = &agregateBoxFixtDef;


    agregateBall.addFixture(agregateBoxFixtDefPtr);



    /*** Chain testing ***/
    b2BodyDef chainBodyDef;
    chainBodyDef.position.Set(-20.0f, 0.0f);
    B2ToSf::SFB2Body agregateChain(world.CreateBody(&chainBodyDef));
    //b2Body* chain_body = world.CreateBody(&chainBodyDef);

    b2Vec2 chain_next(-10,-8);
    b2Vec2 chain_vertices[5];
    chain_vertices[4] = b2Vec2(-10,-6);
    chain_vertices[3] = b2Vec2(-10,0);
    chain_vertices[2] = b2Vec2(-10,10);
    chain_vertices[1] = b2Vec2(10,10);
    chain_vertices[0] = b2Vec2(10,0);
    b2Vec2 chain_prev(10,-1);

    b2ChainShape chain;
    chain.CreateChain(chain_vertices,5, chain_prev, chain_next);
    //chain_body->CreateFixture(&chain,0.0f);


    B2ToSf::SfFixtureGraphical chainAgregateGraphical(4,20,30,sf::Color::White,sf::Color::Red);

    agregateChain.addFixture(&chain,0,chainAgregateGraphical);
    agregateChain.reacquaintSfB2Fixts();




    /*** Chain loop testing ***/

    chain_vertices[4] = b2Vec2(-10,-6);
    chain_vertices[3] = b2Vec2(-10,0);
    chain_vertices[2] = b2Vec2(-10,10);
    chain_vertices[1] = b2Vec2(10,10);
    chain_vertices[0] = b2Vec2(10,0);


    b2BodyDef chain_loop_BodyDef;
    chain_loop_BodyDef.position.Set(-80.0f, 0.0f);
    B2ToSf::SFB2Body agregateLoopedChain(world.CreateBody(&chain_loop_BodyDef));
    //b2Body* chain_loop_body = world.CreateBody(&chain_loop_BodyDef);

    b2ChainShape chain_loop;
    chain_loop.CreateLoop(chain_vertices,5);
    B2ToSf::SfFixtureGraphical chainLoopAgregateGraphical(7,8,30,sf::Color::White,sf::Color::Green);
    agregateLoopedChain.addFixture(&chain_loop,0,chainLoopAgregateGraphical);
    agregateLoopedChain.reacquaintSfB2Fixts();




    /***Edge testing***/
    b2BodyDef ena_crtica_def;
    ena_crtica_def.position.Set(0.0f,-100.0f);
    B2ToSf::SFB2Body agregateEdge(world.CreateBody(&ena_crtica_def));
    //b2Body* ena_crtica = world.CreateBody(&ena_crtica_def);

    b2EdgeShape ena_crtica_shape;

    //ena_crtica_shape.SetTwoSided(b2Vec2(10,-25),b2Vec2(-100,-30));
    ena_crtica_shape.SetOneSided(b2Vec2(14,-25),b2Vec2(10,-25),b2Vec2(-100,-30),b2Vec2(-104,-30));
    B2ToSf::SfFixtureGraphical agregateEdgeGraphical(3,1,0,sf::Color::White,sf::Color::Blue);
    agregateEdge.addFixture(&ena_crtica_shape,0,agregateEdgeGraphical);
    agregateEdge.reacquaintSfB2Fixts();
    //ena_crtica->CreateFixture(&ena_crtica_shape,0.0f);
        /*B2ToSf::EdgyFan ena_crtica_visable = Transl8::line(ena_crtica_shape);
        ena_crtica_visable.setPosition(Transl8::vec2(ena_crtica->GetPosition()));
        ena_crtica_visable.setOutlineThickness(3);
        ena_crtica_visable.setOutlineColor(sf::Color::Blue);*/


    /***Circle testing***/
    b2BodyDef krog_def;
    krog_def.position.Set(0.0f, -100.0f);
    b2Body* krog = world.CreateBody(&krog_def);

    b2CircleShape krog_shape;
    krog_shape.m_radius = 30;
    krog->CreateFixture(&krog_shape,0.0f);

        B2ToSf::EdgyFan krog_visable = Transl8::circle(krog_shape,30);
        krog_visable.setPosition(Transl8::vec2(krog->GetPosition()));
        krog_visable.setOutlineThickness(-30);
        krog_visable.setOutlineColor(sf::Color::Green);



    /** terra nova, more pointy this time... */
    b2BodyDef terraNovaDef;
    terraNovaDef.position.Set(0.0f, -50.0f);
    b2Body* terraNova = world.CreateBody(&terraNovaDef);
    b2PolygonShape terraNovaShape;
    b2Vec2 terraNovaVertices[4];
    terraNovaVertices[0] = b2Vec2(-20,30);
    terraNovaVertices[1] = b2Vec2(10,24);
    terraNovaVertices[2] = b2Vec2(-200,30);
    terraNovaVertices[3] = b2Vec2(100,-5);
    terraNovaShape.Set(terraNovaVertices, 4);

    terraNova->CreateFixture(&terraNovaShape,0.0f);

        B2ToSf::EdgyFan TerNov = Transl8::convexPolygon(terraNovaShape);
        TerNov.setPosition(Transl8::vec2(terraNova->GetPosition()));
        TerNov.setFillColor(sf::Color::Black);
        TerNov.setOutlineColor(sf::Color::Blue);
        TerNov.setOutlineThickness(-5);




    /** Player block **/
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody; // this makes body dynamic i.e. it reacts to forces
    bodyDef.position.Set(0.0f, 4.0f);

    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f); // this is in fact 2x2 box!

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.9f;
    fixtureDef.restitution = 0.1f;
    fixtureDef.isSensor = false;
    //fixtureDef.filter.groupIndex = -8;

    b2Body* body = world.CreateBody(&bodyDef);
    body->CreateFixture(&fixtureDef); // binds fixture to body
    body->SetFixedRotation(true);

        //sf representation
        B2ToSf::EdgyFan dynamicBodyVisable = Transl8::convexPolygon(dynamicBox);


    //Ball and chain

    b2BodyDef ballandchain_def;
    ballandchain_def.type = b2_dynamicBody;
    ballandchain_def.position.Set(-20.0f, 2.0f);

    b2CircleShape ballandchain_circle;
    ballandchain_circle.m_radius = 0.5f;

    b2FixtureDef ballandchain_fixture;
    ballandchain_fixture.shape = &ballandchain_circle;
    ballandchain_fixture.density = 0.9f;
    ballandchain_fixture.friction = 6.0f;
    ballandchain_fixture.restitution  = 0.0f;
    ballandchain_fixture.isSensor = false;
    ballandchain_fixture.userData.pointer = reinterpret_cast<uintptr_t>(&ballandchain_def);

    b2Body* ballandchain = world.CreateBody(&ballandchain_def);
    ballandchain->CreateFixture(&ballandchain_fixture);

        //sf ball and chain
        B2ToSf::EdgyFan ball_and_chain_visable = Transl8::circle(ballandchain_circle);

    b2DistanceJointDef chainofball_def;
    chainofball_def.Initialize(body,ballandchain,body->GetWorldCenter(),ballandchain->GetWorldCenter());
    chainofball_def.collideConnected = true;
    b2RevoluteJoint* chainofball = (b2RevoluteJoint*)world.CreateJoint(&chainofball_def);

    //sensor body

    b2BodyDef sensor_def;
    sensor_def.type = b2_dynamicBody;
    sensor_def.position.Set(0.0f, -0.40f);

    b2PolygonShape sensor_box;
    sensor_box.SetAsBox(0.9f,0.3f);

    b2FixtureDef sensor_fixture_def;
    sensor_fixture_def.shape = &sensor_box;
    sensor_fixture_def.density = 1.2f;
    sensor_fixture_def.friction = 0.9f;
    sensor_fixture_def.restitution = 0.1f;
    sensor_fixture_def.isSensor = true;
    //sensor_fixture_def.filter.groupIndex = -8;

    b2Body* sensor = world.CreateBody(&sensor_def);
    sensor->CreateFixture(&sensor_fixture_def);
    sensor->SetFixedRotation(true);

        B2ToSf::EdgyFan sensor_visable = Transl8::convexPolygon(sensor_box);
        sensor_visable.setFillColor(sf::Color::Red);

    b2WeldJointDef sensor_joint_def;
    sensor_joint_def.Initialize(body,sensor,body->GetWorldCenter());
    sensor_joint_def.localAnchorA = body->GetLocalCenter() - b2Vec2(0.0f,1.0f);
    sensor_joint_def.localAnchorB = sensor->GetLocalCenter();
    sensor_joint_def.collideConnected = false;
    b2WeldJoint* sensor_joint = (b2WeldJoint*)world.CreateJoint(&sensor_joint_def);


    /*b2BodyDef sensor_def;
    sensor_def.type = b2_dynamicBody; // ne vem če je tole smiselno??
    sensor_def.position.Set(0.0f, 5.0f);

    b2PolygonShape sensor_box;
    sensor_box.SetAsBox(1.0f, 0.1f);

    b2FixtureDef sensor_fixture_def;
    sensor_fixture_def.shape = &sensor_box;
    fixtureDef.density = 1.0f;
    sensor_fixture_def.isSensor = false;

    b2Body* sensor = world.CreateBody(& sensor_def);
    sensor->CreateFixture(&sensor_fixture_def);
    body->SetFixedRotation(false);

    b2WeldJointDef sensor_joint_def;
    sensor_joint_def.Initialize(sensor,body,sensor->GetWorldCenter());
    b2WeldJoint* joint = (b2WeldJoint*)world.CreateJoint(&sensor_joint_def);*/







    /***Triangle stripe1***/
    B2ToSf::EdgyStripe ersteTrakc(7);

    ersteTrakc.setPoint(0, sf::Vector2f(0.f*UniversalConstants::kScaling,20.f*UniversalConstants::kScaling));
    ersteTrakc.setPoint(1, sf::Vector2f(0*UniversalConstants::kScaling,25*UniversalConstants::kScaling));
    ersteTrakc.setPoint(2, sf::Vector2f(5*UniversalConstants::kScaling,20*UniversalConstants::kScaling));
    ersteTrakc.setPoint(3, sf::Vector2f(8*UniversalConstants::kScaling,30*UniversalConstants::kScaling));
    ersteTrakc.setPoint(4, sf::Vector2f(20*UniversalConstants::kScaling,0*UniversalConstants::kScaling));
    ersteTrakc.setPoint(5, sf::Vector2f(20*UniversalConstants::kScaling,20*UniversalConstants::kScaling));
    ersteTrakc.setPoint(6, sf::Vector2f(30*UniversalConstants::kScaling,10*UniversalConstants::kScaling));

    ersteTrakc.setFillColor(sf::Color::Red);

    /***Triangle stripe2***/
    B2ToSf::EdgyStripe zweiteTrakc;
    sf::VertexArray zweiteTrakcEdges(sf::PrimitiveType::Points,7);

    zweiteTrakcEdges[0] = sf::Vector2f(0.f*UniversalConstants::kScaling,20.f*UniversalConstants::kScaling);
    zweiteTrakcEdges[1] = sf::Vector2f(0*UniversalConstants::kScaling,25*UniversalConstants::kScaling);
    zweiteTrakcEdges[2] = sf::Vector2f(5*UniversalConstants::kScaling,20*UniversalConstants::kScaling);
    zweiteTrakcEdges[3] = sf::Vector2f(8*UniversalConstants::kScaling,30*UniversalConstants::kScaling);
    zweiteTrakcEdges[4] = sf::Vector2f(20*UniversalConstants::kScaling,0*UniversalConstants::kScaling);
    zweiteTrakcEdges[5] = sf::Vector2f(20*UniversalConstants::kScaling,20*UniversalConstants::kScaling);
    zweiteTrakcEdges[6] = sf::Vector2f(30*UniversalConstants::kScaling,10*UniversalConstants::kScaling);



    zweiteTrakc.setPoints(zweiteTrakcEdges);

    zweiteTrakc.setRotation(180);
    zweiteTrakc.setFillColor(sf::Color::Blue);
    zweiteTrakc.setOutlineColor(sf::Color::Green);
    zweiteTrakc.setOutlineThickness(-20);

    /***Incomplete triangle fan ***/
    B2ToSf::EdgyFan incompleteFan;
    sf::VertexArray incompleteFanVertices(sf::PrimitiveType::TriangleFan,5);

    incompleteFanVertices[0] = sf::Vector2f(0.f*UniversalConstants::kScaling,0.f*UniversalConstants::kScaling);
    incompleteFanVertices[1] = sf::Vector2f(0.0f*UniversalConstants::kScaling,15.0f*UniversalConstants::kScaling);
    incompleteFanVertices[2] = sf::Vector2f(15.0f*UniversalConstants::kScaling,0.f*UniversalConstants::kScaling);
    incompleteFanVertices[3] = sf::Vector2f(0.f*UniversalConstants::kScaling,-15.0f*UniversalConstants::kScaling);
    incompleteFanVertices[4] = sf::Vector2f(-15.0f*UniversalConstants::kScaling,0.f*UniversalConstants::kScaling);
    //incompleteFanVertices[5] = sf::Vector2f(0.f*UniversalConstants::kScaling,15.0f*UniversalConstants::kScaling);
    //incompleteFanVertices[1] = sf::Vector2f(-10.0f*UniversalConstants::kScaling,-5.0f*UniversalConstants::kScaling);
    //incompleteFanVertices[0] = sf::Vector2f(-0.f*UniversalConstants::kScaling,20.0f*UniversalConstants::kScaling);

    incompleteFan.setPoints(incompleteFanVertices);
    incompleteFan.setRotation(45);
    incompleteFan.setOutlineThickness(-30);
    incompleteFan.setOutlineColor(sf::Color::Red);



    /***Complete triangle fan ***/
    B2ToSf::EdgyFan completeFan;
    sf::VertexArray completeFanVertices(sf::PrimitiveType::TriangleFan,6);

    completeFanVertices[0] = sf::Vector2f(0.f*UniversalConstants::kScaling,15.0f*UniversalConstants::kScaling);
    completeFanVertices[1] = sf::Vector2f(0.0f*UniversalConstants::kScaling,15.0f*UniversalConstants::kScaling);
    completeFanVertices[2] = sf::Vector2f(15.0f*UniversalConstants::kScaling,0.f*UniversalConstants::kScaling);
    completeFanVertices[3] = sf::Vector2f(0.f*UniversalConstants::kScaling,-15.0f*UniversalConstants::kScaling);
    completeFanVertices[4] = sf::Vector2f(-15.0f*UniversalConstants::kScaling,0.f*UniversalConstants::kScaling);
    completeFanVertices[5] = sf::Vector2f(0.f*UniversalConstants::kScaling,15.0f*UniversalConstants::kScaling);


    completeFan.setPoints(completeFanVertices);

    completeFan.setRotation(15);
    completeFan.setFillColor(sf::Color::Cyan);
    completeFan.setOutlineThickness(60);
    completeFan.setOutlineColor(sf::Color::Red);
    completeFan.setPosition(sf::Vector2f(-500,0));





    // Running the simulaiton
    float timeStep = 1.0f / 100.0f; // 60 Hz frame rate?
    int32 velocityIterations = 6;
    int32 positionIterations = 2;

    sf::Clock clap;
    sf::Time clapDuration = sf::milliseconds(5);

    sf::Clock tick;
    sf::Time tickDuration = sf::milliseconds(1);



    sf::View observer;
    sf::FloatRect observerRect(sf::Vector2f(0, 0),sf::Vector2f(UniversalConstants::kWindowWidth, UniversalConstants::kWindowHeight));

    observer.reset(observerRect);
    observer.setCenter(dynamicBodyVisable.getPosition());

    //observer.setViewport(sf::FloatRect(0.f,0.f,1.f,1.f));


    worldViable.setView(observer);


    while(worldViable.isOpen())
    {
        sf::Event event;

        while(worldViable.pollEvent(event))
        {
            if(event.type == sf::Event::Closed)
            {
                worldViable.close();
            }
        }

        if(tick.getElapsedTime() >= tickDuration)
        {

            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
            {
                body->ApplyLinearImpulseToCenter(b2Vec2(-0.3,0), true);
            }
            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
            {
                body->ApplyLinearImpulseToCenter(b2Vec2(0.3,0),true);
            }

            if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
            {

                if(sensor->GetContactList() && (sensor->GetContactList())->contact->IsTouching())
                {
                    body->SetLinearVelocity(b2Vec2(body->GetLinearVelocity().x,30));
                    //body->ApplyLinearImpulseToCenter(b2Vec2(0,10),true);
                }

            }



            world.Step(timeStep,velocityIterations,positionIterations);
            tick.restart();
            b2Vec2 position = body->GetPosition();
            float angle = body->GetAngle();
            //printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
            //if(sensor->GetContactList()) std::cout << "true";
            //else std::cout << "false";
            //std::cout <<"\n";

        }

        if(clap.getElapsedTime() >= clapDuration)
        {
            b2Vec2 position = body->GetPosition();
            //float angle = body->GetAngle();

            float angle = Transl8::getRotation(body);

            worldViable.clear();

            dynamicBodyVisable.setPosition(Transl8::vec2(position));
            dynamicBodyVisable.setRotation(angle);

            ball_and_chain_visable.setPosition(Transl8::vec2(ballandchain->GetPosition()));

            sensor_visable.setPosition(Transl8::vec2(sensor->GetPosition()));

            agregateBall.reacquaintSfB2Fixts();



            observer.setCenter(dynamicBodyVisable.getPosition());
            worldViable.setView(observer);
            worldViable.draw(groundBodyVisable);

            worldViable.draw(TerNov);
            //worldViable.draw(ena_crtica_visable);
            worldViable.draw(agregateEdge);
            worldViable.draw(krog_visable);
            worldViable.draw(ersteTrakc);
            worldViable.draw(zweiteTrakc);
            worldViable.draw(incompleteFan);
            worldViable.draw(completeFan);
            worldViable.draw(agregateChain);
            worldViable.draw(agregateLoopedChain);
            worldViable.draw(dynamicBodyVisable);
            worldViable.draw(ball_and_chain_visable);
            worldViable.draw(sensor_visable);
            worldViable.draw(agregateBall);
            worldViable.display();
            clap.restart();
        }


    }
    return 0;
}
