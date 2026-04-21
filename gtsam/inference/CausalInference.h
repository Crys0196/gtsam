/*
 * @Author: Crys0196 aircat110@gmail.com
 * @Date: 2025-12-23 09:35:41
 * @LastEditors: Crys0196 aircat110@gmail.com
 * @LastEditTime: 2025-12-25 13:20:52
 * @FilePath: /gtsam/gtsam/inference/CausalInference.h
 * @Description:Causal inference utilities for Gaussian estimation.
 *
 * Copyright (c) 2025 by Crys0196, All Rights Reserved.
 */
#ifndef _CAUSALINFERENCE_H_
#define _CAUSALINFERENCE_H_

#include <gtsam/inference/Key.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <unordered_map>
#include <map>
#include <vector>

namespace gtsam
{
    namespace ci
    {

        /**
         * @brief Causal inference utilities for Gaussian estimation.
         *
         * CI (Causal Inference) here refers to directional conditional influence
         * inferred from Gaussian elimination. Regression coefficients in Gaussian
         * conditionals and Schur complements are interpreted under a linear–Gaussian
         * structural equation model.
         *
         * This module is stateless and performs no graph modification.
         */
        class CausalInference
        {
        public:
            // ============================================================
            // Gate A: Factor-level conditional information (pre-elimination)
            // ============================================================

            struct FactorScore
            {
                double value; ///< ||ΔΛ_y^{cond}||
                size_t dimY;  ///< dimension of y
            };

            /**
             * @brief Compute conditional information contribution of a factor.
             *
             * Approximates:
             *   ΔΛ_y = J_yᵀJ_y − J_yᵀJ_z (J_zᵀJ_z)⁻¹ J_zᵀJ_y
             *
             * @param factor      Linearized JacobianFactor
             * @param activeKeys  Keys considered as y (variables of interest)
             * @param damping     Diagonal damping for numerical stability
             */
            static FactorScore computeFactorScore(
                const JacobianFactor &factor,
                const KeySet &activeKeys,
                double damping = 1e-6);

            // ============================================================
            // Gate B: Variable-level causal influence (post-elimination)
            // ============================================================

            struct VariableScore
            {
                double value; ///< Σ ||A_ik||
            };

            /**
             * @brief Compute causal influence of a variable on active set.
             *
             * Uses regression coefficients from Gaussian conditionals in the Bayes tree.
             *
             * @param key         Variable to evaluate
             * @param bayesTree   GaussianBayesTree after elimination
             * @param activeKeys  Keys in the active window
             */
            static VariableScore computeVariableScore(
                Key key,
                const GaussianBayesTree &bayesTree,
                const KeySet &activeKeys);

            static std::unordered_map<Key, double>
            computeVariableInfluenceFromActive(
                const GaussianBayesTree &bayesTree,
                const KeySet &activeKeys);

            static std::unordered_map<Key, double>
            computeVariableJacobianInfluence(
                Key x,
                const NonlinearFactorGraph &graph,
                const KeySet &activeKeys);

        private:
            // Utility: compute Frobenius norm safely
            static double frobeniusNorm(const Matrix &M);
        };

    } // namespace ci
} // namespace gtsam

#endif // _CAUSALINFERENCE_H_