//
// Created by wlxing on 2023/12/18.
//
#include "kdtree.h"

#include <glog/logging.h>
#include <execution>
#include <set>

namespace eroam {

    bool KdTree::BuildTree(const CloudPtr &cloud) {
        if (cloud->empty()) {
            return false;
        }

        cloud_.clear();
        cloud_.resize(cloud->size());
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            cloud_[i] = ToVec3f(cloud->points[i]);
        }

        Clear();
        Reset();

        IndexVec idx(cloud->size());
        for (int i = 0; i < cloud->points.size(); ++i) {
            idx[i] = i;
        }

        Insert(idx, root_.get());
        return true;
    }

    void KdTree::Insert(const IndexVec &points, KdTreeNode *node) {
        nodes_.insert({node->id_, node});

        if (points.empty()) {
            return;
        }

        if (points.size() == 1) {
            size_++;
            node->point_idx_ = points[0];
            return;
        }

        IndexVec left, right;
        if (!FindSplitAxisAndThresh(points, node->axis_index_, node->split_thresh_, left, right)) {
            size_++;
            node->point_idx_ = points[0];
            return;
        }

        const auto create_if_not_empty = [&node, this](KdTreeNode *&new_node, const IndexVec &index) {
            if (!index.empty()) {
                new_node = new KdTreeNode;
                new_node->id_ = tree_node_id_++;
                Insert(index, new_node);
            }
        };

        create_if_not_empty(node->left_, left);
        create_if_not_empty(node->right_, right);
    }

    bool KdTree::GetClosestPoint(const PointType &pt, std::vector<int> &closest_idx, int k) {
        if (k > size_) {
            LOG(ERROR) << "cannot set k larger than cloud size: " << k << ", " << size_;
            return false;
        }
        k_ = k;

        std::priority_queue<NodeAndDistance> knn_result;
        Knn(ToVec3f(pt), root_.get(), knn_result);

        // sort the result, and return
        closest_idx.resize(knn_result.size());
        for (int i = closest_idx.size() - 1; i >= 0; --i) {
            // insert from the end
            closest_idx[i] = knn_result.top().node_->point_idx_;
            knn_result.pop();
        }
        return true;
    }

    bool KdTree::GetClosestPointMT(const CloudPtr &cloud, std::vector<std::pair<size_t, size_t>> &matches, int k) {
        matches.resize(cloud->size() * k);

        // index[i] = i
        std::vector<int> index(cloud->size());
        for (int i = 0; i < cloud->points.size(); ++i) {
            index[i] = i;
        }

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [this, &cloud, &matches, &k](int idx) {
            std::vector<int> closest_idx;
            GetClosestPoint(cloud->points[idx], closest_idx, k);
            for (int i = 0; i < k; ++i) {
                matches[idx * k + i].second = idx;
                if (i < closest_idx.size()) {
                    matches[idx * k + i].first = closest_idx[i];
                } else {
                    matches[idx * k + i].first = math::kINVALID_ID;
                }
            }
        });

        return true;
    }

    void KdTree::Knn(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const {
        if (node->IsLeaf()) {
            // if node is leaf, check if it can be inserted
            ComputeDisForLeaf(pt, node, knn_result);
            return;
        }

        // check which side the pt is in, and search the side first
        // then check if the other side needs to be searched
        KdTreeNode *this_side, *that_side;
        if (pt[node->axis_index_] < node->split_thresh_) {
            this_side = node->left_;
            that_side = node->right_;
        } else {
            this_side = node->right_;
            that_side = node->left_;
        }

        Knn(pt, this_side, knn_result);
        // note: compare with itself
        if (NeedExpand(pt, node, knn_result)) {
            Knn(pt, that_side, knn_result);
        }
    }

    bool KdTree::NeedExpand(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const {
        if (knn_result.size() < k_) {
            return true;
        }

        if (approximate_) {
            float d = pt[node->axis_index_] - node->split_thresh_;
            if ((d * d) < knn_result.top().distance2_ * alpha_) {
                return true;
            } else {
                return false;
            }
        } else {
            // compute the distance between pt and the split plane, if it is smaller than the max distance, then search
            float d = pt[node->axis_index_] - node->split_thresh_;
            if ((d * d) < knn_result.top().distance2_) {
                return true;
            } else {
                return false;
            }
        }
    }

    void KdTree::ComputeDisForLeaf(const Vec3f &pt, KdTreeNode *node,
                                   std::priority_queue<NodeAndDistance> &knn_result) const {
        // compute the distance between pt and the leaf node,
        // and insert it into the result if it is smaller than the max
        float dis2 = Dis2(pt, cloud_[node->point_idx_]);
        if (knn_result.size() < k_) {
            // if results is not enough, insert it
            knn_result.emplace(node, dis2);
        } else {
            // if results is enough, compare it with the max distance
            if (dis2 < knn_result.top().distance2_) {
                knn_result.emplace(node, dis2);
                knn_result.pop();
            }
        }
    }

    bool
    KdTree::FindSplitAxisAndThresh(const IndexVec &point_idx, int &axis, float &th, IndexVec &left, IndexVec &right) {
        // compute the variance of each axis
        Vec3f var;
        Vec3f mean;
        math::ComputeMeanAndCovDiag(point_idx, mean, var, [this](int idx) { return cloud_[idx]; });
        int max_i, max_j;
        var.maxCoeff(&max_i, &max_j);
        axis = max_i;
        th = mean[axis];

        for (const auto &idx: point_idx) {
            if (cloud_[idx][axis] < th) {
                // median may be rounded down
                left.emplace_back(idx);
            } else {
                right.emplace_back(idx);
            }
        }

        // boundary condition check: the input points are equal to the same value, the above judgment is >=,
        // so they all enter the right side, this situation does not need to continue to expand, just set the current
        if (point_idx.size() > 1 && (left.empty() || right.empty())) {
            return false;
        }

        return true;
    }

    void KdTree::Reset() {
        tree_node_id_ = 0;
        root_.reset(new KdTreeNode());
        root_->id_ = tree_node_id_++;
        size_ = 0;
    }

    void KdTree::Clear() {
        for (const auto &np: nodes_) {
            if (np.second != root_.get()) {
                delete np.second;
            }
        }

        nodes_.clear();
        root_ = nullptr;
        size_ = 0;
        tree_node_id_ = 0;
    }

    void KdTree::PrintAll() {
        for (const auto &np: nodes_) {
            auto node = np.second;
            if (node->left_ == nullptr && node->right_ == nullptr) {
                LOG(INFO) << "leaf node: " << node->id_ << ", idx: " << node->point_idx_;
            } else {
                LOG(INFO) << "node: " << node->id_ << ", axis: " << node->axis_index_ << ", th: "
                          << node->split_thresh_;
            }
        }
    }

}  // namespace eroam
