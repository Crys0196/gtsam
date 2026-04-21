/*
 * @Author: Crys0196 aircat110@gmail.com
 * @Date: 2025-04-14 10:32:29
 * @LastEditors: Crys0196 aircat110@gmail.com
 * @LastEditTime: 2025-05-10 21:38:15
 * @FilePath: /gtsam/gtsam/inference/ReverseMaximin.h
 * @Description:A new ordering and new data structure to modify the iSAM2.
 *
 * Copyright (c) 2025 by Crys0196, All Rights Reserved.
 */
#ifndef _REVERSEMAXIMIN_H_
#define _REVERSEMAXIMIN_H_

#include <gtsam/inference/Key.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_set>
#include <unordered_map>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Values-inl.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <stdexcept>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
// CRYS START RMM
/**
 * Reverse Maximum-Minimum Distance (RMM) Ordering.
 * This heuristic attempts to maximize the distance between the eliminated node
 * and the remaining graph, reducing worst-case fill-in.
 */

using namespace gtsam;

namespace gtsam
{

    /** Node distance wrapper for heap */
    struct DistanceNode
    {
        Key key;
        mutable double distance;
        using Handle = boost::heap::fibonacci_heap<DistanceNode>::handle_type;
        bool operator<(const DistanceNode &other) const
        {

            return distance < other.distance;
        }
    };

    Eigen::VectorXd encoding(const Values &values, Key key)
    {
        Eigen::VectorXd vec = Eigen::VectorXd::Zero(15);

        Symbol symbol(key);
        char prefix = symbol.chr();

        switch (prefix)
        {
        case 'v':
        {
            if (!values.exists<Point3>(key))
                throw std::runtime_error("Expected Point3 for key starting with 'v'");
            Point3 p = values.at<Point3>(key);
            vec.segment<3>(0) = p; // slot [0,3)
            break;
        }
        case 'x':
        {
            if (!values.exists<Pose3>(key))
                throw std::runtime_error("Expected Pose3 for key starting with 'x'");
            Pose3 pose = values.at<Pose3>(key);
            vec.segment<3>(3) = pose.translation();    // slot [3,6)
            vec.segment<3>(6) = pose.rotation().rpy(); // slot [6,9)
            break;
        }
        case 'b':
        {
            if (!values.exists<imuBias::ConstantBias>(key))
                throw std::runtime_error("Expected ConstantBias for key starting with 'b'");
            imuBias::ConstantBias bias = values.at<imuBias::ConstantBias>(key);
            vec.segment<3>(9) = bias.accelerometer(); // slot [9,12)
            vec.segment<3>(12) = bias.gyroscope();    // slot [12,15)
            break;
        }
        default:
            throw std::runtime_error("Unknown key prefix: expected 'v', 'x', or 'b'");
        }

        return vec;
    }
    /** Compute Euclidean squared distance between variable keys  */
    double dist2(Key a, Key b, const Values &values)
    {
        if (!values.exists(a) || !values.exists(b))
        {
            throw std::runtime_error("dist2: One or both keys not found in values.");
        }

        Eigen::VectorXd va = encoding(values, a);
        Eigen::VectorXd vb = encoding(values, b);

        return (va - vb).squaredNorm();
        throw std::runtime_error("dist2: Unsupported or mismatched types between keys.");
    }
    /** Reverse Maximin Ordering */
    template <class FACTOR_GRAPH>
    Ordering ReverseMaximinOrdering(const FACTOR_GRAPH &graph, const Values &values, Key startKey = 1, double rho = 2.0)
    {
        if (graph.empty())
            return Ordering();

        VariableIndex varIndex(graph);

        const size_t nVars = varIndex.size();
        if (nVars == 0)
        {
            return Ordering();
        }

        if (nVars == 1)
        {
            return Ordering(KeyVector(1, varIndex.begin()->first));
        }
        Ordering ordering;
        std::unordered_set<Key> eliminated;

        // Fibonacci_heap
        boost::heap::fibonacci_heap<DistanceNode> mutableHeap;
        std::unordered_map<Key, DistanceNode::Handle> handle_map;

        // std::unordered_map<Key, Key> parents;It seems that we don't need the parents map

        // Initialize the heap with distances from the startKey

        for (const auto &entry : varIndex)
        {
            Key key = entry.first;

            double d = (key == startKey) ? std::numeric_limits<double>::max() : std::sqrt(dist2(key, startKey, values));

            auto handle = mutableHeap.push({key, d});
            handle_map[key] = handle;
        }

        while (!mutableHeap.empty())
        {
            DistanceNode pivot = mutableHeap.top();
            mutableHeap.pop();

            ordering.push_back(pivot.key);
            eliminated.insert(pivot.key);
            handle_map.erase(pivot.key);

            // BUG Here we just consider the non-zero parts to decrease calculation
            const auto &factorIndices = varIndex[pivot.key];
            for (size_t factorIdx : factorIndices)
            {
                const auto &factor = graph[factorIdx];
                for (Key neighbor : factor->keys())
                {
                    if (eliminated.count(neighbor))
                        continue;
                    double d = std::sqrt(dist2(neighbor, pivot.key, values));
                    auto handle = handle_map[neighbor];
                    if (d <= rho * pivot.distance && d < (*handle).distance)
                    {
                        (*handle).distance = d;
                        mutableHeap.update(handle); // Update heap with the new distance
                    }
                }
            }
        }
        std::reverse(ordering.begin(), ordering.end());
        return ordering;
    }
} // namespace gtsam

#endif // _REVERSEMAXIMIN_H_