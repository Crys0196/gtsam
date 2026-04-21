/*
 * @Author: Crys0196 aircat110@gmail.com
 * @Date: 2025-12-23 12:32:57
 * @LastEditors: Crys0196 aircat110@gmail.com
 * @LastEditTime: 2025-12-25 14:17:02
 * @FilePath: /gtsam/gtsam_unstable/nonlinear/AdaptiveLagSmoother.cpp
 * @Description:Using Causal Inference to adjust the time window of variable elimination.
 *
 * Copyright (c) 2025 by Crys0196, All Rights Reserved.
 */
#include "AdaptiveLagSmoother.h"
#include <iostream>

namespace gtsam
{

    AdaptiveLagSmoother::AdaptiveLagSmoother(
        double baseLag,
        double minLag,
        double factorThreshold,
        double variableThreshold,
        double activeTimeRadius)
        : baseLag_(baseLag),
          minLag_(minLag),
          factorThreshold_(factorThreshold),
          variableThreshold_(variableThreshold),
          activeTimeRadius_(activeTimeRadius),
          smoother_(baseLag) {}

    KeySet AdaptiveLagSmoother::inferImplicitActiveKeys(
        const KeyTimestampMap &timestamps,
        double currentTimestamp) const
    {

        KeySet active;
        for (const auto &[k, t] : timestamps)
        {
            if (currentTimestamp - t <= activeTimeRadius_)
            {
                active.insert(k);
            }
        }
        return active;
    }

    /* =========================
     * Gate A: factor admission
     * ========================= */
    void AdaptiveLagSmoother::gateFactors(
        NonlinearFactorGraph &newFactors,
        const KeySet &activeKeys)
    {

        NonlinearFactorGraph kept;

        for (const auto &f : newFactors)
        {
            if (!f)
                continue;

            // Linearize nonlinear factor
            GaussianFactor::shared_ptr gf = f->linearize(Values());
            auto jf = boost::dynamic_pointer_cast<JacobianFactor>(gf);

            // Keep non-Jacobian or untestable factors conservatively
            if (!jf)
            {
                kept.push_back(f);
                continue;
            }

            auto score = ci_.computeFactorScore(*jf, activeKeys);
            if (score.value >= factorThreshold_)
                kept.push_back(f);
        }

        newFactors = kept;
    }

    /* =========================
     * Gate B: timestamp aging
     * ========================= */
    void AdaptiveLagSmoother::adaptTimestamps(
        KeyTimestampMap &timestamps,
        const NonlinearFactorGraph &graph,
        const KeySet &activeKeys,
        double currentTimestamp)

    {
        if (activeKeys.empty())
            return;

        for (auto &[x, tx] : timestamps)
        {

            // Never age active variables
            if (activeKeys.count(x))
                continue;

            // Compute CI_B(x -> active)
            auto influence =
                ci_.computeVariableJacobianInfluence(
                    x, graph, activeKeys);

            double ci_score = 0.0;
            for (const auto &[_, v] : influence)
                ci_score += v;

            if (ci_score < variableThreshold_)
            {
                double decay =
                    (baseLag_ - minLag_) *
                    std::max(0.0, 1.0 - ci_score / variableThreshold_);
                tx -= decay;
            }
        }
    }

    /* =========================
     * Main update
     * ========================= */
    FixedLagSmoother::Result AdaptiveLagSmoother::update(
        NonlinearFactorGraph &newFactors,
        Values &newValues,
        KeyTimestampMap &timestamps)
    {

        // Determine current time
        double currentTimestamp = getCurrentTimestamp();
        for (const auto &[_, t] : timestamps)
            currentTimestamp = std::max(currentTimestamp, t);

        // Infer active keys
        KeySet activeKeys =
            inferImplicitActiveKeys(timestamps, currentTimestamp);

        // Gate A
        gateFactors(newFactors, activeKeys);

        // IFLS update (graph grows here)
        auto result =
            smoother_.update(newFactors, newValues, timestamps);

        // Gate B (after update, factor-based)
        adaptTimestamps(
            timestamps,
            newFactors, // current local graph view
            activeKeys,
            currentTimestamp);

        return result;
    }

} // namespace gtsam_unstable
