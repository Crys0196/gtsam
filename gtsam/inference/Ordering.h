/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.h
 * @brief   Variable ordering for the elimination algorithm
 * @author  Richard Roberts
 * @author  Andrew Melim
 * @author  Frank Dellaert
 * @date    Sep 2, 2010
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/MetisIndex.h>
#include <gtsam/base/FastSet.h>

#include <algorithm>
#include <vector>
#include <queue>
namespace gtsam
{

  class Ordering : public KeyVector
  {
  protected:
    typedef KeyVector Base;

  public:
    /// Type of ordering to use
    enum OrderingType
    {
      COLAMD,
      METIS,
      NATURAL,
      CUSTOM,
      DRMM
    };

    typedef Ordering This;                      ///< Typedef to this class
    typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

    /// Create an empty ordering
    GTSAM_EXPORT
    Ordering()
    {
    }

    /// Create from a container
    template <typename KEYS>
    explicit Ordering(const KEYS &keys) : Base(keys.begin(), keys.end())
    {
    }

    /// Create an ordering using iterators over keys
    template <typename ITERATOR>
    Ordering(ITERATOR firstKey, ITERATOR lastKey) : Base(firstKey, lastKey)
    {
    }

    /// Add new variables to the ordering as ordering += key1, key2, ...  Equivalent to calling
    /// push_back.
    boost::assign::list_inserter<boost::assign_detail::call_push_back<This>> operator+=(
        Key key)
    {
      return boost::assign::make_list_inserter(
          boost::assign_detail::call_push_back<This>(*this))(key);
    }

    /// Invert (not reverse) the ordering - returns a map from key to order position
    FastMap<Key, size_t> invert() const;

    /// @name Fill-reducing Orderings @{

    /// Compute a fill-reducing ordering using COLAMD from a factor graph (see details for note on
    /// performance). This internally builds a VariableIndex so if you already have a VariableIndex,
    /// it is faster to use COLAMD(const VariableIndex&)
    template <class FACTOR_GRAPH>
    static Ordering Colamd(const FACTOR_GRAPH &graph)
    {
      if (graph.empty())
        return Ordering();
      else
        return Colamd(VariableIndex(graph));
    }

    /// Compute a fill-reducing ordering using COLAMD from a VariableIndex.
    static GTSAM_EXPORT Ordering Colamd(const VariableIndex &variableIndex);

    /// Compute a fill-reducing ordering using constrained COLAMD from a factor graph (see details
    /// for note on performance).  This internally builds a VariableIndex so if you already have a
    /// VariableIndex, it is faster to use COLAMD(const VariableIndex&).  This function constrains
    /// the variables in \c constrainLast to the end of the ordering, and orders all other variables
    /// before in a fill-reducing ordering.  If \c forceOrder is true, the variables in \c
    /// constrainLast will be ordered in the same order specified in the KeyVector \c
    /// constrainLast.   If \c forceOrder is false, the variables in \c constrainLast will be
    /// ordered after all the others, but will be rearranged by CCOLAMD to reduce fill-in as well.
    template <class FACTOR_GRAPH>
    static Ordering ColamdConstrainedLast(const FACTOR_GRAPH &graph,
                                          const KeyVector &constrainLast, bool forceOrder = false)
    {
      if (graph.empty())
        return Ordering();
      else
        return ColamdConstrainedLast(VariableIndex(graph), constrainLast, forceOrder);
    }

    /// Compute a fill-reducing ordering using constrained COLAMD from a VariableIndex.  This
    /// function constrains the variables in \c constrainLast to the end of the ordering, and orders
    /// all other variables before in a fill-reducing ordering.  If \c forceOrder is true, the
    /// variables in \c constrainLast will be ordered in the same order specified in the KeyVector
    /// \c constrainLast.   If \c forceOrder is false, the variables in \c constrainLast will be
    /// ordered after all the others, but will be rearranged by CCOLAMD to reduce fill-in as well.
    static GTSAM_EXPORT Ordering ColamdConstrainedLast(
        const VariableIndex &variableIndex, const KeyVector &constrainLast,
        bool forceOrder = false);

    /// Compute a fill-reducing ordering using constrained COLAMD from a factor graph (see details
    /// for note on performance).  This internally builds a VariableIndex so if you already have a
    /// VariableIndex, it is faster to use COLAMD(const VariableIndex&).  This function constrains
    /// the variables in \c constrainLast to the end of the ordering, and orders all other variables
    /// before in a fill-reducing ordering.  If \c forceOrder is true, the variables in \c
    /// constrainFirst will be ordered in the same order specified in the KeyVector \c
    /// constrainFirst.   If \c forceOrder is false, the variables in \c constrainFirst will be
    /// ordered before all the others, but will be rearranged by CCOLAMD to reduce fill-in as well.
    template <class FACTOR_GRAPH>
    static Ordering ColamdConstrainedFirst(const FACTOR_GRAPH &graph,
                                           const KeyVector &constrainFirst, bool forceOrder = false)
    {
      if (graph.empty())
        return Ordering();
      else
        return ColamdConstrainedFirst(VariableIndex(graph), constrainFirst, forceOrder);
    }

    /// Compute a fill-reducing ordering using constrained COLAMD from a VariableIndex.  This
    /// function constrains the variables in \c constrainFirst to the front of the ordering, and
    /// orders all other variables after in a fill-reducing ordering.  If \c forceOrder is true, the
    /// variables in \c constrainFirst will be ordered in the same order specified in the
    /// KeyVector \c constrainFirst.   If \c forceOrder is false, the variables in \c
    /// constrainFirst will be ordered before all the others, but will be rearranged by CCOLAMD to
    /// reduce fill-in as well.
    static GTSAM_EXPORT Ordering ColamdConstrainedFirst(
        const VariableIndex &variableIndex,
        const KeyVector &constrainFirst, bool forceOrder = false);

    /// Compute a fill-reducing ordering using constrained COLAMD from a factor graph (see details
    /// for note on performance).  This internally builds a VariableIndex so if you already have a
    /// VariableIndex, it is faster to use COLAMD(const VariableIndex&).  In this function, a group
    /// for each variable should be specified in \c groups, and each group of variables will appear
    /// in the ordering in group index order.  \c groups should be a map from Key to group index.
    /// The group indices used should be consecutive starting at 0, but may appear in \c groups in
    /// arbitrary order.  Any variables not present in \c groups will be assigned to group 0.  This
    /// function simply fills the \c cmember argument to CCOLAMD with the supplied indices, see the
    /// CCOLAMD documentation for more information.
    template <class FACTOR_GRAPH>
    static Ordering ColamdConstrained(const FACTOR_GRAPH &graph,
                                      const FastMap<Key, int> &groups)
    {
      if (graph.empty())
        return Ordering();
      else
        return ColamdConstrained(VariableIndex(graph), groups);
    }

    /// Compute a fill-reducing ordering using constrained COLAMD from a VariableIndex.  In this
    /// function, a group for each variable should be specified in \c groups, and each group of
    /// variables will appear in the ordering in group index order.  \c groups should be a map from
    /// Key to group index. The group indices used should be consecutive starting at 0, but may
    /// appear in \c groups in arbitrary order.  Any variables not present in \c groups will be
    /// assigned to group 0.  This function simply fills the \c cmember argument to CCOLAMD with the
    /// supplied indices, see the CCOLAMD documentation for more information.
    static GTSAM_EXPORT Ordering ColamdConstrained(
        const VariableIndex &variableIndex, const FastMap<Key, int> &groups);

    /// Return a natural Ordering. Typically used by iterative solvers
    template <class FACTOR_GRAPH>
    static Ordering Natural(const FACTOR_GRAPH &fg)
    {
      KeySet src = fg.keys();
      KeyVector keys(src.begin(), src.end());
      std::stable_sort(keys.begin(), keys.end());
      return Ordering(keys);
    }

    /// METIS Formatting function
    template <class FACTOR_GRAPH>
    static GTSAM_EXPORT void CSRFormat(std::vector<int> &xadj,
                                       std::vector<int> &adj, const FACTOR_GRAPH &graph);

    /// Compute an ordering determined by METIS from a VariableIndex
    static GTSAM_EXPORT Ordering Metis(const MetisIndex &met);

    template <class FACTOR_GRAPH>
    static Ordering Metis(const FACTOR_GRAPH &graph)
    {
      if (graph.empty())
        return Ordering();
      else
        return Metis(MetisIndex(graph));
    }

    /// @}

    // CRYS START RMM
    /**
     * Reverse Maximum-Minimum Distance (RMM) Ordering.
     * This heuristic attempts to maximize the distance between the eliminated node
     * and the remaining graph, reducing worst-case fill-in.
     */
    // static GTSAM_EXPORT Ordering degreeRMM(const VariableIndex &variableIndex)
    // {
    //   const size_t nVars = variableIndex.size();
    //   if (nVars == 0)
    //   {
    //     return Ordering();
    //   }

    //   if (nVars == 1)
    //   {
    //     return Ordering(KeyVector(1, variableIndex.begin()->first));
    //   }
    //   // get degrees
    //   std::map<gtsam::Key, int> degrees;
    //   for (const auto &key_factors : variableIndex)
    //   {
    //     const Key &key = key_factors.first;
    //     const FactorIndices &column = key_factors.second;
    //     degrees[key] = column.size();
    //   }

    //   auto comparator = [&](gtsam::Key a, gtsam::Key b)
    //   {
    //     return degrees[a] != degrees[b] ? degrees[a] < degrees[b] : a > b; // max degree heap and min key if equal
    //   };
    //   std::priority_queue<gtsam::Key, std::vector<gtsam::Key>, decltype(comparator)> queue(comparator);
    //   for (const auto &entry : degrees)
    //   {
    //     queue.push(entry.first);
    //   }

    //   // 生成反向Maximin顺序
    //   gtsam::Ordering ordering;
    //   std::set<gtsam::Key> marked;
    //   while (!queue.empty())
    //   {
    //     gtsam::Key current = queue.top();
    //     queue.pop();
    //     if (variableIndex.empty(current))
    //     {
    //       continue; // skip empty factor
    //     }
    //     if (marked.count(current))
    //       continue;

    //     // 正向插入，后续反转
    //     ordering.push_back(current);
    //     marked.insert(current);

    //     // 更新邻居的度（模拟消元操作）
    //     // for (const auto &factor : graph)
    //     // {
    //     //   if (factor->exists(current))
    //     //   {
    //     //     for (gtsam::Key neighbor : factor->keys())
    //     //     {
    //     //       if (!marked.count(neighbor))
    //     //       {
    //     //         degrees[neighbor]--;
    //     //         // 重新插入队列以更新优先级
    //     //         queue.push(neighbor);
    //     //       }
    //     //     }
    //     //   }
    //     // }

    //     for (Key factorIndex : variableIndex[current])
    //     {
    //       // Here we have the index of factor(or we call it neighbor index)
    //       Key neighbor = factorIndex;
    //       if (neighbor == current || marked.count(neighbor))
    //         continue;
    //       degrees[neighbor]--;
    //       queue.push(neighbor);
    //     }
    //   }
    //   std::reverse(ordering.begin(), ordering.end());
    //   return ordering;
    // }
    template <class FACTOR_GRAPH>
    static GTSAM_EXPORT Ordering degreeRMM(const FACTOR_GRAPH &graph)
    {
      if (graph.empty())
        return Ordering();

      VariableIndex variableIndex(graph);

      const size_t nVars = variableIndex.size();
      if (nVars == 0)
      {
        return Ordering();
      }

      if (nVars == 1)
      {
        return Ordering(KeyVector(1, variableIndex.begin()->first));
      }
      // get degrees
      std::map<gtsam::Key, int> degrees;
      for (const auto &key_factors : variableIndex)
      {
        const Key &key = key_factors.first;
        const FactorIndices &column = key_factors.second;
        degrees[key] = column.size();
      }

      auto comparator = [&](gtsam::Key a, gtsam::Key b)
      {
        return degrees[a] != degrees[b] ? degrees[a] < degrees[b] : a > b; // max degree heap and min key if equal
      };
      std::priority_queue<gtsam::Key, std::vector<gtsam::Key>, decltype(comparator)> queue(comparator);
      for (const auto &entry : degrees)
      {
        queue.push(entry.first);
      }

      // 生成反向Maximin顺序
      gtsam::Ordering ordering;
      std::set<gtsam::Key> marked;
      while (!queue.empty())
      {
        gtsam::Key current = queue.top();
        queue.pop();
        if (variableIndex.empty(current))
        {
          continue; // skip empty factor
        }
        if (marked.count(current))
          continue;

        // 正向插入，后续反转
        ordering.push_back(current);
        marked.insert(current);

        // 更新邻居的度（模拟消元操作）
        // for (const auto &factor : graph)
        // {
        //   if (factor->exists(current))
        //   {
        //     for (gtsam::Key neighbor : factor->keys())
        //     {
        //       if (!marked.count(neighbor))
        //       {
        //         degrees[neighbor]--;
        //         // 重新插入队列以更新优先级
        //         queue.push(neighbor);
        //       }
        //     }
        //   }
        // }

        for (size_t factorIndex : variableIndex[current])
        {
          const auto &factor = graph.at(factorIndex);
          for (Key neighbor : factor->keys())
          {
            if (neighbor == current || marked.count(neighbor))
              continue;
            degrees[neighbor]--;
            queue.push(neighbor);
          }
        }
      }
      std::reverse(ordering.begin(), ordering.end());
      return ordering;
    }

    // CRYS END RMM

    /// @name Named Constructors @{

    template <class FACTOR_GRAPH>
    static Ordering Create(OrderingType orderingType,
                           const FACTOR_GRAPH &graph)
    {
      if (graph.empty())
        return Ordering();

      switch (orderingType)
      {
      case COLAMD:
        return Colamd(graph);
      case METIS:
        return Metis(graph);
      case NATURAL:
        return Natural(graph);
      case CUSTOM:
        throw std::runtime_error(
            "Ordering::Create error: called with CUSTOM ordering type.");
      case DRMM:
        return degreeRMM(graph);
      default:
        throw std::runtime_error(
            "Ordering::Create error: called with unknown ordering type.");
      }
    }

    /// @}

    /// @name Testable @{

    GTSAM_EXPORT
    void print(const std::string &str = "", const KeyFormatter &keyFormatter =
                                                DefaultKeyFormatter) const;

    GTSAM_EXPORT
    bool equals(const Ordering &other, double tol = 1e-9) const;

    /// @}

  private:
    /// Internal COLAMD function
    static GTSAM_EXPORT Ordering ColamdConstrained(
        const VariableIndex &variableIndex, std::vector<int> &cmember);

    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version)
    {
      ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
  };

  /// traits
  template <>
  struct traits<Ordering> : public Testable<Ordering>
  {
  };

}
