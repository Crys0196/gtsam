#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "CausalInference.h"
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/linear/JacobianFactor.h>
#include <Eigen/Dense>
#include <limits>

namespace gtsam
{
    namespace ci
    {

        // ============================================================
        // Utility helpers
        // ============================================================

        double CausalInference::frobeniusNorm(const Matrix &M)
        {
            return std::sqrt((M.array() * M.array()).sum());
        }

        // ============================================================
        // Gate A: Factor-level conditional information
        // ============================================================

        CausalInference::FactorScore
        CausalInference::computeFactorScore(
            const JacobianFactor &factor,
            const KeySet &activeKeys,
            double damping)
        {

            std::vector<Matrix> Jy_blocks, Jz_blocks;
            size_t dimY = 0, dimZ = 0;

            for (auto it = factor.begin(); it != factor.end(); ++it)
            {
                Key k = *it;
                const auto Ablock = factor.getA(it); // constABlock

                // Convert ABlock to dense Matrix (safe)
                Matrix A = Ablock.matrix();

                if (activeKeys.exists(k))
                {
                    Jy_blocks.push_back(A);
                    dimY += A.cols();
                }
                else
                {
                    Jz_blocks.push_back(A);
                    dimZ += A.cols();
                }
            }

            if (dimY == 0)
                return {0.0, 0};

            const size_t m = factor.rows();
            Matrix Jy(m, dimY), Jz(m, dimZ);

            size_t col = 0;
            for (const auto &A : Jy_blocks)
            {
                Jy.middleCols(col, A.cols()) = A;
                col += A.cols();
            }

            col = 0;
            for (const auto &A : Jz_blocks)
            {
                Jz.middleCols(col, A.cols()) = A;
                col += A.cols();
            }

            Matrix Lambda_yy = Jy.transpose() * Jy;

            if (dimZ > 0)
            {
                Matrix Lambda_zz = Jz.transpose() * Jz;
                Matrix Lambda_yz = Jy.transpose() * Jz;

                Lambda_zz.diagonal().array() += damping;

                Matrix schur =
                    Lambda_yy -
                    Lambda_yz * Lambda_zz.ldlt().solve(Lambda_yz.transpose());

                return {frobeniusNorm(schur), dimY};
            }

            return {frobeniusNorm(Lambda_yy), dimY};
        }

        // ============================================================
        // Gate B: Variable-level causal influence
        // ============================================================
        static void accumulateInfluence(
            const GaussianBayesTreeClique::shared_ptr &clique,
            Key targetKey,
            const KeySet &activeKeys,
            double &influence)
        {

            if (!clique)
                return;

            const auto &conditional = clique->conditional();
            if (conditional)
            {

                // Check if this conditional affects active variables
                bool affectsActive = false;
                for (Key f : conditional->frontals())
                {
                    if (activeKeys.exists(f))
                    {
                        affectsActive = true;
                        break;
                    }
                }

                if (affectsActive)
                {
                    // Iterate over parents using iterators
                    for (auto it = conditional->beginParents();
                         it != conditional->endParents(); ++it)
                    {

                        if (*it == targetKey)
                        {
                            const auto Ablock = conditional->getA(it);
                            influence += std::sqrt(Ablock.squaredNorm());
                        }
                    }
                }
            }

            // Recurse
            for (const auto &child : clique->children)
            {
                accumulateInfluence(child, targetKey, activeKeys, influence);
            }
        }
        CausalInference::VariableScore
        CausalInference::computeVariableScore(
            Key key,
            const GaussianBayesTree &bayesTree,
            const KeySet &activeKeys)
        {

            double influence = 0.0;

            for (const auto &root : bayesTree.roots())
            {
                accumulateInfluence(root, key, activeKeys, influence);
            }

            return {influence};
        }

        std::unordered_map<Key, double>
        CausalInference::computeVariableInfluenceFromActive(
            const GaussianBayesTree &bayesTree,
            const KeySet &activeKeys)
        {

            std::unordered_map<Key, double> influence;

            for (Key active : activeKeys)
            {

                // Each frontal variable has exactly one clique
                auto clique = bayesTree.clique(active);
                if (!clique)
                    continue;

                const auto &conditional = clique->conditional();
                if (!conditional)
                    continue;

                // Sanity: active must be frontal here
                // (optional assertion)

                for (auto it = conditional->beginParents();
                     it != conditional->endParents(); ++it)
                {

                    const auto Ablock = conditional->getA(it);
                    influence[*it] += std::sqrt(Ablock.squaredNorm());
                }
            }

            return influence;
        }

        std::unordered_map<Key, double>
        CausalInference::computeVariableJacobianInfluence(
            Key x,
            const NonlinearFactorGraph &graph,
            const KeySet &activeKeys)
        {

            std::unordered_map<Key, double> influence;

            // Initialize map for active keys
            for (Key y : activeKeys)
            {
                influence[y] = 0.0;
            }

            // Iterate over all factors
            for (const NonlinearFactor::shared_ptr &factor : graph)
            {

                // Skip null or unrelated factors early
                if (factor->keys().empty())
                    continue;

                // Linearize factor
                GaussianFactor::shared_ptr gaussian = factor->linearize(Values());
                if (!gaussian)
                    continue;

                // We only support JacobianFactor
                auto jf = boost::dynamic_pointer_cast<JacobianFactor>(gaussian);
                if (!jf)
                    continue;

                // Locate iterator for x
                auto it_x = jf->begin();
                for (; it_x != jf->end(); ++it_x)
                {
                    if (*it_x == x)
                        break;
                }

                if (it_x == jf->end())
                    continue;

                const JacobianFactor::ABlock Ax = jf->getA(it_x);

                // Loop over other variables in this factor
                for (auto it_y = jf->begin(); it_y != jf->end(); ++it_y)
                {

                    Key y = *it_y;

                    if (y == x)
                        continue;

                    if (activeKeys.count(y) == 0)
                        continue;

                    const JacobianFactor::ABlock Ay = jf->getA(it_y);

                    // Accumulate Frobenius norm of Ay^T * Ax
                    influence[y] += (Ay.transpose() * Ax).norm();
                }
            }

            return influence;
        }
    } // namespace ci
} // namespace gtsam