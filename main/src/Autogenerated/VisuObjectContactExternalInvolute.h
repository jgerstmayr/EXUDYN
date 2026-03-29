/** ***********************************************************************************************
* @class        VisualizationObjectContactExternalInvolute
* @brief        Visualization object for the external involute gear contact connector
*
************************************************************************************************ */

#ifndef VISUALIZATIONOBJECTCONTACTEXTERNALINVOLUTE__H
#define VISUALIZATIONOBJECTCONTACTEXTERNALINVOLUTE__H

#include <ostream>

#include "Utilities/ReleaseAssert.h"
#include "Utilities/BasicDefinitions.h"
#include "System/ItemIndices.h"

class VisualizationSettings;
class VisualizationSystem;

class VisualizationObjectContactExternalInvolute : public VisualizationObject
{
protected:
    Float4 color;

public:
    VisualizationObjectContactExternalInvolute()
    {
        show = true;
        color = Float4({-1.f, -1.f, -1.f, -1.f});
    }

    virtual void UpdateGraphics(const VisualizationSettings& visualizationSettings, VisualizationSystem* vSystem, Index itemNumber) override;

    virtual bool IsConnector() const override
    {
        return true;
    }

    void SetColor(const Float4& value) { color = value; }
    const Float4& GetColor() const { return color; }
    Float4& GetColor() { return color; }
};

#endif
