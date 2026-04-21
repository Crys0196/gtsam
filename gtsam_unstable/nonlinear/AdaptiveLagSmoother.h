/*
 * @Author: Crys0196 aircat110@gmail.com
 * @Date: 2025-12-23 12:32:44
 * @LastEditors: Crys0196 aircat110@gmail.com
 * @LastEditTime: 2025-12-25 14:16:47
 * @FilePath: /gtsam/gtsam_unstable/nonlinear/AdaptiveLagSmoother.h
 * @Description: Using Causal Inference to adjust the time window of variable elimination.
 *
 * Copyright (c) 2025 by Crys0196, All Rights Reserved.
 */
#ifndef _ADAPTIVELAGSMOOTHER_H_
#define _ADAPTIVELAGSMOOTHER_H_

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "gtsam/inference/CausalInference.h"

#include <unordered_map>

namespace gtsam
{

    class GTSAM_UNSTABLE_EXPORT AdaptiveLagSmoother : public FixedLagSmoother
    {
    public:
        AdaptiveLagSmoother(
            double baseLag,
            double minLag,
            double ciFactorThreshold,
            double ciVariableThreshold,
            double activeTimeRadius);

        FixedLagSmoother::Result update(
            NonlinearFactorGraph &newFactors,
            Values &newValues,
            KeyTimestampMap &timestamps);

        Values calculateEstimate() const override
        {
            return isam_.calculateEstimate();
        }

        template <class VALUE>
        VALUE calculateEstimate(Key key) const
        {
            return isam_.calculateEstimate<VALUE>(key);
        }

    protected:
        ISAM2 isam_;
        ISAM2Result isamResult_;

    private:
        IncrementalFixedLagSmoother smoother_;
        ci::CausalInference ci_;

        double baseLag_ = 5.0;
        double minLag_ = 2.0;
        double factorThreshold_ = 1e-3;
        double variableThreshold_ = 1e-2;
        double activeTimeRadius_ = 1.0;

        // Gate A
        void gateFactors(
            NonlinearFactorGraph &newFactors,
            const KeySet &activeKeys);

        // Gate B
        void adaptTimestamps(
            KeyTimestampMap &timestamps,
            const NonlinearFactorGraph &graph,
            const KeySet &activeKeys,
            double currentTimestamp);
        gtsam::KeySet inferImplicitActiveKeys(
            const KeyTimestampMap &timestamps,
            double currentTimestamp) const;
    };

} // namespace gtsam

#endif // _ADAPTIVELAGSMOOTHER_H_