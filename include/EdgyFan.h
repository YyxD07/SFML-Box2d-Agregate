#ifndef EDGYFAN_H
#define EDGYFAN_H


#include "Graphics.hpp"
#include "box2d.h"

#include "ConstantSettings.h"

#include "clockwiseNormal.h"

#include "SFShapeBase.h"



namespace B2ToSf
{

class EdgyFan: public SFShapeBase
{
    public:
        EdgyFan(std::size_t vertex_count = 0);
        EdgyFan(const EdgyFan& edgy_fan_to_copy);
        virtual ~EdgyFan();


    private:

        void m_makeOutline(); // Needs at least 4 vertices
};
}// namespace B2ToSf

#endif // EDGYFAN_H
