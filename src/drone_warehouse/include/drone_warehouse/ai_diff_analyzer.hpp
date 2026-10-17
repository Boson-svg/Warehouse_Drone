#pragma once

#include "drone_warehouse/models.hpp"

#include <QString>
#include <QVector>

class AiDiffAnalyzer
{
public:
    static SlotRuleAnalysis analyzeSlot(const SlotAnalysisInput &input);
    static QVector<SlotRuleAnalysis> analyzeAll(const QVector<SlotAnalysisInput> &inputs);

private:
    static bool hasManualData(const SlotAnalysisInput &input);
    static bool hasObservedData(const SlotAnalysisInput &input);
    static bool hasPartialObservedData(const SlotAnalysisInput &input);
    static bool hasPositionOnly(const SlotAnalysisInput &input);
    static QString slotDisplayName(const SlotAnalysisInput &input);
};