//
// Created by wlxing on 2023/12/18.
//

#ifndef FX_LIDAR_ODOMETRY_KDTREE_H
#define FX_LIDAR_ODOMETRY_KDTREE_H

#include "eigen_types.h"
#include "math_utils.h"
#include "point_types.h"
#include <glog/logging.h>
#include <map>
#include <queue>

namespace eroam {

    /**
     * KdTreeNode, a binary tree structure, used raw pointer internally, and a shared_ptr for root externally
     */
    struct KdTreeNode {
        int id_ = -1;
        int point_idx_ = 0;            // index of the point
        int axis_index_ = 0;           // axis of the split
        float split_thresh_ = 0.0;     // split position
        KdTreeNode *left_ = nullptr;   // left child
        KdTreeNode *right_ = nullptr;  // right child

        bool IsLeaf() const { return left_ == nullptr && right_ == nullptr; }  // 是否为叶子
    };

    /**
     * NodeAndDistance, a struct to record the node and its distance to the query point
     */
    struct NodeAndDistance {
        NodeAndDistance(KdTreeNode *node, float dis2) : node_(node), distance2_(dis2) {}

        KdTreeNode *node_ = nullptr;
        float distance2_ = 0;  // the square of distance, used for comparison

        bool operator<(const NodeAndDistance &other) const { return distance2_ < other.distance2_; }
    };


    /**
     * KdTree
     */
    class KdTree {
    public:
        explicit KdTree() = default;

        ~KdTree() { Clear(); }

        bool BuildTree(const CloudPtr &cloud);

        /**
         * get k nearest neighbors of a point
         * @param pt
         * @param closest_idx
         * @param k
         * @return
         */
        bool GetClosestPoint(const PointType &pt, std::vector<int> &closest_idx, int k = 5);

        /**
         * get k nearest neighbors of a point with multi-threading
         * @param cloud
         * @param matches
         * @param k
         * @return
         */
        bool GetClosestPointMT(const CloudPtr &cloud, std::vector<std::pair<size_t, size_t>> &matches, int k = 5);

        /**
         * set the multiple of the nearest neighbor search
         * @param use_ann
         * @param alpha
         */
        void SetEnableANN(bool use_ann = true, float alpha = 0.1) {
            approximate_ = use_ann;
            alpha_ = alpha;
        }

        /**
         * get the number of nodes
         * @return
         */
        size_t size() const { return size_; }

        /**
         * clear the tree
         */
        void Clear();

        /**
         * print all nodes
         */
        void PrintAll();

    private:

        /**
         * insert points at node
         * @param points
         * @param node
         */
        void Insert(const IndexVec &points, KdTreeNode *node);


        /**
         * compute the split axis and threshold of the point set
         * @param point_idx
         * @param axis
         * @param th
         * @param left
         * @param right
         * @return
         */
        bool FindSplitAxisAndThresh(const IndexVec &point_idx, int &axis, float &th, IndexVec &left, IndexVec &right);

        void Reset();

        /**
         * compute the square of distance between two points
         * @param p1
         * @param p2
         * @return
         */
        static inline float Dis2(const Vec3f &p1, const Vec3f &p2) { return (p1 - p2).squaredNorm(); }


        /**
         * check the knn of the given point on the kdtree node, can be called recursively
         * @param pt
         * @param node
         * @param result
         */
        void Knn(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &result) const;


        /**
         * for leaf node, compute the distance between it and the query point, try to put it into the result
         * @param pt
         * @param node
         * @param result
         */
        void ComputeDisForLeaf(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &result) const;


        /**
         * check if the node needs to be expanded
         * @param pt
         * @param node
         * @param knn_result
         * @return
         */
        bool NeedExpand(const Vec3f &pt, KdTreeNode *node, std::priority_queue<NodeAndDistance> &knn_result) const;

        int k_ = 5;                                     // the number of nearest neighbors
        std::shared_ptr<KdTreeNode> root_ = nullptr;    // the root node
        std::vector<Vec3f> cloud_;                      // the input point cloud
        std::unordered_map<int, KdTreeNode *> nodes_;   // the map of node id and node pointer

        size_t size_ = 0;                               // the number of leaf nodes
        int tree_node_id_ = 0;                          // the id of node

        bool approximate_ = true;                       // approximate nearest neighbor search
        float alpha_ = 0.1;
    };

}  // namespace eroam

#endif  // FX_LIDAR_ODOMETRY_KDTREE_H
