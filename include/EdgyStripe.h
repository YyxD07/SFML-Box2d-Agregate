#ifndef EDGYSTRIPE_H
#define EDGYSTRIPE_H

#include "Graphics.hpp"
#include "box2d.h"

#include "ConstantSettings.h"

#include "clockwiseNormal.h"

#include "SFShapeBase.h"



//setFillColor setOutlineColor setOutlineThickness


namespace B2ToSf
{

class EdgyStripe: public SFShapeBase
{
public:
    EdgyStripe(std::size_t vertec_count = 0);
    EdgyStripe(const EdgyStripe& edgy_stripe_to_copy);
    virtual ~EdgyStripe();



private:

    void m_makeOutline(); // Needs at least 4 vertices
};

}//namespace B2ToSf

#endif // EDGYSTRIPE_H
