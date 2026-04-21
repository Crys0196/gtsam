/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IncrementalFixedLagSmoother.cpp
 * @brief   An iSAM2-based fixed-lag smoother. To the extent possible, this class mimics the iSAM2
 * interface. However, additional parameters, such as the smoother lag and the timestamp associated
 * with each variable are needed.
 *
 * @author  Michael Kaess, Stephen Williams
 * @date    Oct 14, 2012
 */

#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/BayesTreeMarginalizationHelper.h>
#include <gtsam/base/debug.h>
#include <fstream>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/inference/ReverseMaximin.h>

namespace gtsam
{

  /* ************************************************************************* */
  // crys start OUTPUT MATRIX
  void saveMatrix(const Eigen::MatrixXd &infoMatrix, const std::string &filename)
  {
    std::ofstream file(filename);
    if (!file.is_open())
    {
      std::cerr << "Error: Could not open file " << filename << " for writing!" << std::endl;
      return;
    }

    // 设置输出格式，保证矩阵数据对齐
    file << std::fixed << std::setprecision(6);

    for (int i = 0; i < infoMatrix.rows(); ++i)
    {
      for (int j = 0; j < infoMatrix.cols(); ++j)
      {
        file << infoMatrix(i, j);
        if (j < infoMatrix.cols() - 1)
          file << ", "; // CSV格式
      }
      file << "\n";
    }
    file.close();
  }

  Eigen::MatrixXd extractInfoMatrix(const gtsam::ISAM2 &solver)
  {
    gtsam::GaussianFactorGraph::shared_ptr gfg = solver.getFactorsUnsafe().linearize(solver.getLinearizationPoint());
    Eigen::MatrixXd infoMatrix = gfg->hessian().first;

    // Eigen:: MatrixXd AugmentedJ=gfg->augmentedJacobian(); Augmented Jacobian [A|b]

    return infoMatrix;
  }

  Eigen::MatrixXd extractRMatrix(const gtsam::ISAM2 &solver)
  {
    const gtsam::Values &linearization_point = solver.getLinearizationPoint();
    gtsam::GaussianFactorGraph::shared_ptr gfg = solver.getFactorsUnsafe().linearize(linearization_point);

    gtsam::Ordering ordering = Ordering::Create(Ordering::NATURAL, *gfg);
    // Values values = solver.calculateEstimate();
    // if (values.empty())
    // {
    //   std::cout << "Values Empty!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    //   return Eigen::MatrixXd::Zero(1, 1);
    // }

    // KeyVector Keys = values.keys();
    // Key startKey = Keys[0];

    // gtsam::Ordering ordering = ReverseMaximinOrdering(*gfg, values, startKey);
    const gtsam::GaussianBayesNet::shared_ptr bayesNet_ptr = gfg->eliminateSequential(ordering);

    std::map<gtsam::Key, int> dimensions;
    for (gtsam::Key key : ordering)
    {
      if (linearization_point.exists(key))
      {
        dimensions[key] = linearization_point.at(key).dim(); // 需要实际类型判断
      }
      else
      {
        throw std::runtime_error("Key not found in linearization point");
      }
    }

    int total_dim = 0;
    std::map<gtsam::Key, int> row_offsets;
    for (gtsam::Key key : ordering)
    {
      row_offsets[key] = total_dim;
      total_dim += dimensions[key];
    }

    // 初始化R矩阵
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(total_dim, total_dim);

    // 填充R矩阵
    for (const auto &conditional : *bayesNet_ptr)
    {
      const gtsam::KeyVector &keys = conditional->keys();
      gtsam::Key front_key = keys.front();

      // 当前条件概率对应的行范围
      int rows = dimensions[front_key];
      int row_start = row_offsets[front_key];

      Eigen::MatrixXd R1 = conditional->R();
      // 遍历所有父节点，填充对应列
      int current_col = 0;
      for (gtsam::Key key : keys)
      {
        int cols = dimensions[key];
        int col_start = row_offsets[key];
        R.block(row_start, col_start, rows, cols) = R1.block(0, current_col, rows, cols);
        current_col += cols;
      }
    }

    return R;
  }

  // 获取 Hessian 近似信息矩阵并保存
  void exportInformationMatrix(const Eigen::MatrixXd &infoMatrix, const std::string &label)
  {
    std::string filenameIM = "/home/crys0196/InfoMatrix/" + label + "_InformationMatrix.csv";
    saveMatrix(infoMatrix, filenameIM);
  }


  // 获取 R 矩阵并保存
  void exportRMatrix(const Eigen::MatrixXd &RMatrix, const std::string &label)
  {
    std::string filenameIM = "/home/crys0196/RMatrix/Natural/" + label + "_RMatrix.csv";
    saveMatrix(RMatrix, filenameIM);
  }
  // crys end OUTPUT MATRIX

  void recursiveMarkAffectedKeys(const Key &key,
                                 const ISAM2Clique::shared_ptr &clique, std::set<Key> &additionalKeys)
  {

    // Check if the separator keys of the current clique contain the specified key
    if (std::find(clique->conditional()->beginParents(),
                  clique->conditional()->endParents(), key) != clique->conditional()->endParents())
    {

      // Mark the frontal keys of the current clique
      for (Key i : clique->conditional()->frontals())
      {
        additionalKeys.insert(i);
      }

      // Recursively mark all of the children
      for (const ISAM2Clique::shared_ptr &child : clique->children)
      {
        recursiveMarkAffectedKeys(key, child, additionalKeys);
      }
    }
    // If the key was not found in the separator/parents, then none of its children can have it either
  }

  /* ************************************************************************* */
  void IncrementalFixedLagSmoother::print(const std::string &s,
                                          const KeyFormatter &keyFormatter) const
  {
    FixedLagSmoother::print(s, keyFormatter);
    // TODO: What else to print?
  }

  /* ************************************************************************* */
  bool IncrementalFixedLagSmoother::equals(const FixedLagSmoother &rhs,
                                           double tol) const
  {
    const IncrementalFixedLagSmoother *e =
        dynamic_cast<const IncrementalFixedLagSmoother *>(&rhs);
    return e != nullptr && FixedLagSmoother::equals(*e, tol) && isam_.equals(e->isam_, tol);
  }

  /* ************************************************************************* */
  FixedLagSmoother::Result IncrementalFixedLagSmoother::update(
      const NonlinearFactorGraph &newFactors, const Values &newTheta,
      const KeyTimestampMap &timestamps, const FactorIndices &factorsToRemove)
  {

    const bool debug = ISDEBUG("IncrementalFixedLagSmoother update");

    if (debug)
    {
      std::cout << "IncrementalFixedLagSmoother::update() Start" << std::endl;
      PrintSymbolicTree(isam_, "Bayes Tree Before Update:");
      std::cout << "END" << std::endl;
    }

    FastVector<size_t> removedFactors;
    boost::optional<FastMap<Key, int>> constrainedKeys = boost::none;

    // Update the Timestamps associated with the factor keys
    updateKeyTimestampMap(timestamps);

    // Get current timestamp
    double current_timestamp = getCurrentTimestamp();

    if (debug)
      std::cout << "Current Timestamp: " << current_timestamp << std::endl;

    // Find the set of variables to be marginalized out
    KeyVector marginalizableKeys = findKeysBefore(
        current_timestamp - smootherLag_);

    if (debug)
    {
      std::cout << "Marginalizable Keys: ";
      for (Key key : marginalizableKeys)
      {
        std::cout << DefaultKeyFormatter(key) << " ";
      }
      std::cout << std::endl;
    }


    // Force iSAM2 to put the marginalizable variables at the beginning
    createOrderingConstraints(marginalizableKeys, constrainedKeys);

    if (debug)
    {
      std::cout << "Constrained Keys: ";
      if (constrainedKeys)
      {
        for (FastMap<Key, int>::const_iterator iter = constrainedKeys->begin();
             iter != constrainedKeys->end(); ++iter)
        {
          std::cout << DefaultKeyFormatter(iter->first) << "(" << iter->second
                    << ")  ";
        }
      }
      std::cout << std::endl;
    }

    // Mark additional keys between the marginalized keys and the leaves
    std::set<Key> additionalKeys;
    for (Key key : marginalizableKeys)
    {
      ISAM2Clique::shared_ptr clique = isam_[key];
      for (const ISAM2Clique::shared_ptr &child : clique->children)
      {
        recursiveMarkAffectedKeys(key, child, additionalKeys);
      }
    }
    KeyList additionalMarkedKeys(additionalKeys.begin(), additionalKeys.end());

    // Crys start

    // exportInformationMatrix(extractInfoMatrix(isam_), "BeforeOptimization" + std::to_string(current_timestamp));

    // exportRMatrix(extractRMatrix(isam_), "BeforeOptimization" + std::to_string(current_timestamp));
    //  Update iSAM2
    isamResult_ = isam_.update(newFactors, newTheta,
                               factorsToRemove, constrainedKeys, boost::none, additionalMarkedKeys);

    std::cout << "Variables relinearized: " << isamResult_.variablesRelinearized << std::endl;
    std::cout << "New factors added: " << isamResult_.newFactorsIndices.size() << std::endl;

    // exportInformationMatrix(extractInfoMatrix(isam_), "AfterOptimization" + std::to_string(current_timestamp));
    // exportRMatrix(extractRMatrix(isam_), "AfterOptimization" + std::to_string(current_timestamp));
    // Crys end

    if (debug)
    {
      PrintSymbolicTree(isam_,
                        "Bayes Tree After Update, Before Marginalization:");
      std::cout << "END" << std::endl;
    }

    // crys start
    // PrintSymbolicTree(isam_, "Bayes Tree After Update, Before Marginalization:");
    // std::cout << "END" << std::endl;
    // if (!isam_.roots().empty())
    // {
    //   isam_.saveGraph("/home/crys0196/Graphs/Bayes_" + std::to_string(current_timestamp) + ".dot");
    // }

    // crys end
    //  Marginalize out any needed variables

    if (marginalizableKeys.size() > 0)
    {
      FastList<Key> leafKeys(marginalizableKeys.begin(),
                             marginalizableKeys.end());
      isam_.marginalizeLeaves(leafKeys);
    }

    // Remove marginalized keys from the KeyTimestampMap
    eraseKeyTimestampMap(marginalizableKeys);

    if (debug)
    {
      PrintSymbolicTree(isam_, "Final Bayes Tree:");
      std::cout << "END" << std::endl;
    }

    // Crys 2024-10-17 Print out the graph files.
    // Crys start

    // PrintSymbolicTree(isam_, "Final Bayes Tree:");
    // std::cout << "END" << std::endl;
    // if(!isam_.roots().empty())
    // {
    //   isam_.saveGraph("/home/crys0196/Graphs/mBayes_"+std::to_string(current_timestamp)+".dot");
    // }
    // std::ofstream osma("/home/crys0196/Graphs/mNG_"+std::to_string(current_timestamp)+".dot");
    // isam_.getFactorsUnsafe().saveGraph(osma);
    // osma.close();
    // Crys end

    // TODO: Fill in result structure
    Result result;
    result.iterations = 1;
    result.linearVariables = 0;
    result.nonlinearVariables = 0;
    result.error = 0;

    if (debug)
      std::cout << "IncrementalFixedLagSmoother::update() Finish" << std::endl;

    return result;
  }

  /* ************************************************************************* */
  void IncrementalFixedLagSmoother::eraseKeysBefore(double timestamp)
  {
    TimestampKeyMap::iterator end = timestampKeyMap_.lower_bound(timestamp);
    TimestampKeyMap::iterator iter = timestampKeyMap_.begin();
    while (iter != end)
    {
      keyTimestampMap_.erase(iter->second);
      timestampKeyMap_.erase(iter++);
    }
  }

  /* ************************************************************************* */
  void IncrementalFixedLagSmoother::createOrderingConstraints(
      const KeyVector &marginalizableKeys,
      boost::optional<FastMap<Key, int>> &constrainedKeys) const
  {
    if (marginalizableKeys.size() > 0)
    {
      constrainedKeys = FastMap<Key, int>();
      // Generate ordering constraints so that the marginalizable variables will be eliminated first
      // Set all variables to Group1
      for (const TimestampKeyMap::value_type &timestamp_key : timestampKeyMap_)
      {
        constrainedKeys->operator[](timestamp_key.second) = 1;
      }
      // Set marginalizable variables to Group0
      for (Key key : marginalizableKeys)
      {
        constrainedKeys->operator[](key) = 0;
      }
    }
  }

  /* ************************************************************************* */
  void IncrementalFixedLagSmoother::PrintKeySet(const std::set<Key> &keys,
                                                const std::string &label)
  {
    std::cout << label;
    for (Key key : keys)
    {
      std::cout << " " << DefaultKeyFormatter(key);
    }
    std::cout << std::endl;
  }

  /* ************************************************************************* */
  void IncrementalFixedLagSmoother::PrintSymbolicFactor(
      const GaussianFactor::shared_ptr &factor)
  {
    std::cout << "f(";
    for (Key key : factor->keys())
    {
      std::cout << " " << DefaultKeyFormatter(key);
    }
    std::cout << " )" << std::endl;
  }

  /* ************************************************************************* */
  void IncrementalFixedLagSmoother::PrintSymbolicGraph(
      const GaussianFactorGraph &graph, const std::string &label)
  {
    std::cout << label << std::endl;
    for (const GaussianFactor::shared_ptr &factor : graph)
    {
      PrintSymbolicFactor(factor);
    }
  }

  /* ************************************************************************* */
  void IncrementalFixedLagSmoother::PrintSymbolicTree(const ISAM2 &isam,
                                                      const std::string &label)
  {
    std::cout << label << std::endl;
    if (!isam.roots().empty())
    {
      for (const ISAM2::sharedClique &root : isam.roots())
      {
        PrintSymbolicTreeHelper(root);
      }
    }
    else
      std::cout << "{Empty Tree}" << std::endl;
  }

  /* ************************************************************************* */
  void IncrementalFixedLagSmoother::PrintSymbolicTreeHelper(
      const ISAM2Clique::shared_ptr &clique, const std::string indent)
  {

    // Print the current clique
    std::cout << indent << "P( ";
    for (Key key : clique->conditional()->frontals())
    {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    if (clique->conditional()->nrParents() > 0)
      std::cout << "| ";
    for (Key key : clique->conditional()->parents())
    {
      std::cout << DefaultKeyFormatter(key) << " ";
    }
    std::cout << ")" << std::endl;

    // Recursively print all of the children
    for (const ISAM2Clique::shared_ptr &child : clique->children)
    {
      PrintSymbolicTreeHelper(child, indent + " ");
    }
  }

  /* ************************************************************************* */
} /// namespace gtsam
