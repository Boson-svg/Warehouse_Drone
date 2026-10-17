#include "drone_warehouse/ai_diff_analyzer.hpp"

namespace
{
QString makeSlotLabel(const SlotAnalysisInput &input)
{
    return QString("%1 %2 R%3C%4")
        .arg(input.shelf_name)
        .arg(input.side)
        .arg(input.row + 1)
        .arg(input.col + 1);
}
}

bool AiDiffAnalyzer::hasManualData(const SlotAnalysisInput &input)
{
    return !input.manual_category_id.isEmpty() || !input.manual_package_id.isEmpty();
}

bool AiDiffAnalyzer::hasObservedData(const SlotAnalysisInput &input)
{
    return !input.observed_category_id.isEmpty() || !input.observed_package_id.isEmpty();
}

bool AiDiffAnalyzer::hasPartialObservedData(const SlotAnalysisInput &input)
{
    const bool has_category = !input.observed_category_id.isEmpty();
    const bool has_package = !input.observed_package_id.isEmpty();
    return has_category != has_package;
}

bool AiDiffAnalyzer::hasPositionOnly(const SlotAnalysisInput &input)
{
    return input.observed_category_id.isEmpty() &&
           input.observed_package_id.isEmpty() &&
           !input.observed_slot_code.isEmpty();
}

QString AiDiffAnalyzer::slotDisplayName(const SlotAnalysisInput &input)
{
    return makeSlotLabel(input);
}

SlotRuleAnalysis AiDiffAnalyzer::analyzeSlot(const SlotAnalysisInput &input)
{
    SlotRuleAnalysis result;
    result.input = input;

    result.has_manual_data = hasManualData(input);
    result.has_observed_data = hasObservedData(input);
    result.has_partial_observed_data = hasPartialObservedData(input);
    result.has_position_only = hasPositionOnly(input);

    result.package_match = !input.manual_package_id.isEmpty() &&
                           !input.observed_package_id.isEmpty() &&
                           input.manual_package_id == input.observed_package_id;

    result.category_match = !input.manual_category_id.isEmpty() &&
                            !input.observed_category_id.isEmpty() &&
                            input.manual_category_id == input.observed_category_id;

    result.exact_match = result.package_match && result.category_match;

    const QString slot_name = slotDisplayName(input);

    if (!result.has_manual_data && !result.has_observed_data && !result.has_position_only)
    {
        result.status = SlotDiffStatus::Empty;
        result.summary = slot_name + "：无台账，无巡检结果";
        result.reason = "当前槽位没有人工台账，也没有巡检识别结果。";
        result.priority = 0;
        result.should_revisit = false;
        return result;
    }

    if (result.exact_match)
    {
        result.status = SlotDiffStatus::Matched;
        result.summary = slot_name + "：台账与巡检一致";
        result.reason = "类别号与包裹号均匹配。";
        result.priority = 5;
        result.should_revisit = false;
        return result;
    }

    if (result.has_manual_data && !result.has_observed_data && !result.has_position_only)
    {
        result.status = SlotDiffStatus::ManualOnly;
        result.summary = slot_name + "：台账有数据，但巡检未识别到";
        result.reason = "人工台账存在，但巡检结果为空。";
        result.priority = 70;
        result.should_revisit = true;
        return result;
    }

    if (!result.has_manual_data && result.has_observed_data)
    {
        result.status = SlotDiffStatus::ObservedOnly;
        result.summary = slot_name + "：巡检识别到货物，但台账为空";
        result.reason = "疑似漏登记、临时放置或错放。";
        result.priority = 85;
        result.should_revisit = true;
        return result;
    }

    if (result.has_position_only)
    {
        result.status = SlotDiffStatus::PositionOnly;
        result.summary = slot_name + "：仅识别到位置，没有识别到货物身份";
        result.reason = "位置码有效，但包裹号/类别号均缺失。";
        result.priority = 75;
        result.should_revisit = true;
        return result;
    }

    if (result.has_partial_observed_data)
    {
        result.status = SlotDiffStatus::PartialObserved;
        result.summary = slot_name + "：巡检结果不完整";
        result.reason = "仅识别出包裹号或类别号的一部分信息。";
        result.priority = input.has_image ? 55 : 78;
        result.should_revisit = true;
        return result;
    }

    if (result.has_observed_data && !input.has_image)
    {
        result.status = SlotDiffStatus::ObservedWithoutImage;
        result.summary = slot_name + "：巡检有识别结果，但本次无图";
        result.reason = "识别结果可用，但本次巡检没有图片证据。";
        result.priority = 60;
        result.should_revisit = true;
        return result;
    }

    result.status = SlotDiffStatus::Mismatch;
    result.summary = slot_name + "：台账与巡检不一致";
    result.reason = "包裹号或类别号存在冲突。";
    result.priority = 95;
    result.should_revisit = true;
    return result;
}

QVector<SlotRuleAnalysis> AiDiffAnalyzer::analyzeAll(const QVector<SlotAnalysisInput> &inputs)
{
    QVector<SlotRuleAnalysis> results;
    results.reserve(inputs.size());

    for (const SlotAnalysisInput &input : inputs)
    {
        results.push_back(analyzeSlot(input));
    }

    return results;
}