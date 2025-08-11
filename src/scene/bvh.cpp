#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
  namespace SceneObjects {

    BVHAccel::BVHAccel(const std::vector<Primitive*>& _primitives,
      size_t max_leaf_size) {

      primitives = std::vector<Primitive*>(_primitives);
      root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
    }

    BVHAccel::~BVHAccel() {
      if (root)
        delete root;
      primitives.clear();
    }

    BBox BVHAccel::get_bbox() const { return root->bb; }

    void BVHAccel::draw(BVHNode* node, const Color& c, float alpha) const {
      if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
          (*p)->draw(c, alpha);
        }
      }
      else {
        draw(node->l, c, alpha);
        draw(node->r, c, alpha);
      }
    }

    void BVHAccel::drawOutline(BVHNode* node, const Color& c, float alpha) const {
      if (node->isLeaf()) {
        for (auto p = node->start; p != node->end; p++) {
          (*p)->drawOutline(c, alpha);
        }
      }
      else {
        drawOutline(node->l, c, alpha);
        drawOutline(node->r, c, alpha);
      }
    }

    BVHNode* BVHAccel::construct_bvh(std::vector<Primitive*>::iterator start,
      std::vector<Primitive*>::iterator end,
      size_t max_leaf_size) {

      // TODO (Part 2.1):
      // Construct a BVH from the given vector of primitives and maximum leaf
      // size configuration. The starter code build a BVH aggregate with a
      // single leaf node (which is also the root) that encloses all the
      // primitives.

       //Base case: Create LEAF node
      if ((end - start) <= max_leaf_size) {
        //cout << "LEAF NODE created with " << (end - start) << " primitives" << endl;
        BBox bbox;
        for (auto p = start; p != end; p++) {
          BBox bb = (*p)->get_bbox();
          bbox.expand(bb);
        }
        BVHNode* leaf = new BVHNode(bbox);
        leaf->start = start;
        leaf->end = end;
        return leaf;
      }

      // compute the max extent on x, y, z and pick the largest
      BBox bbox_root;
      BBox bbox_cen;
      Vector3D cen_sum(0, 0, 0);
      int num_p = 0;

      for (auto p = start; p != end; p++) {
        BBox bb = (*p)->get_bbox();
        Vector3D cen = (*p)->get_bbox().centroid();
        cen_sum += cen;
        bbox_cen.expand(cen);
        bbox_root.expand(bb);
        num_p += 1;
      }
      // get the extent: max-min
      Vector3D dis = bbox_cen.extent;
      //cout << "ext: " << dis << endl;
      //cout << "num pri " << num_p << endl;
      // get the extent in x, y, z
      double x_ext = dis.x;
      double y_ext = dis.y;
      double z_ext = dis.z;

      int split_axis = 0;  // 0=x, 1=y, 2=z
      if (y_ext > x_ext && y_ext > z_ext) split_axis = 1;
      else if (z_ext > x_ext && z_ext > y_ext) split_axis = 2;
      //cout << "split_ext: " << split_axis << endl;

      //the average of centroids along the axis with max extent
      double split = (split_axis == 0) ? cen_sum.x / num_p :
        (split_axis == 1) ? cen_sum.y / num_p :
        cen_sum.z / num_p;


      // split into left and right node
      std::vector<Primitive*> left_primitives, right_primitives;

      for (auto p = start; p != end; p++) {
        Vector3D cen = (*p)->get_bbox().centroid();
        if (cen[split_axis] <= split) {
          left_primitives.push_back(*p);
        }
        else {
          right_primitives.push_back(*p);
        }
      }
      //cout << "left_primitives=" << left_primitives.size()
      //<< ", right_primitives=" << right_primitives.size() << endl;

      auto it = start;
      for (auto p : left_primitives) *it++ = p;
      auto split_iter = it;
      for (auto p : right_primitives) *it++ = p;

      if (split_iter == start || split_iter == end) {
        //cout << "DEGENERATE SPLIT! split_iter == "
          //<< (split_iter == start ? "start" : "end") << endl;
      }

      if (split_iter == start || split_iter == end) {
        // Degenerate split - all primitives on one side
        // Force creation of a leaf node
        BBox bbox;
        for (auto p = start; p != end; p++) {
          bbox.expand((*p)->get_bbox());
        }
        BVHNode* leaf = new BVHNode(bbox);
        leaf->start = start;
        leaf->end = end;
        return leaf;
      }

      //cout << "Making recursive calls: left_size=" << (split_iter - start)
      //  << ", right_size=" << (end - split_iter) << endl;

      BVHNode* left_child = construct_bvh(start, split_iter, max_leaf_size);
      BVHNode* right_child = construct_bvh(split_iter, end, max_leaf_size);

      BVHNode* root_box = new BVHNode(bbox_root);
      root_box->l = left_child;
      root_box->r = right_child;

      return root_box;

      // Original simple version - just create one big leaf
      //BBox bbox;
      //for (auto p = start; p != end; p++) {
      //  BBox bb = (*p)->get_bbox();
      //  bbox.expand(bb);
      //}

      //BVHNode* node = new BVHNode(bbox);
      //node->start = start;
      //node->end = end;
      //return node;
  }

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  //for (auto p : primitives) {
  //  total_isects++;
  //  if (p->has_intersection(ray))
  //    return true;
  //}
  //return false;

   //Test if ray intersects this node's bounding box
  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;  // Ray misses bounding box - skip entire subtree
  }

  // Check if the intersection is within the ray's valid range
  if (t1 < ray.min_t || t0 > ray.max_t) {
    return false;
  }

  // If this is a leaf node - test primitives
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {
        return true;  // Found intersection - can return immediately
      }
    }
    return false;
  }
  else {
    // Internal node - recursively test children
    if (node->l && has_intersection(ray, node->l)) {
      return true;  // Short-circuit: return as soon as we find any hit
    }
    if (node->r && has_intersection(ray, node->r)) {
      return true;
    }
    return false;
  }
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.


  // brute-force technique
  //bool hit = false;
  //for (auto p : primitives) {
  //  total_isects++;
  //  hit = p->intersect(ray, i) || hit;
  //}
  //return hit;
   
  //cout << "intersect called on " << (node->isLeaf() ? "LEAF" : "INTERNAL")
  //  << " node" << endl;
  // Test if ray intersects this node's bounding box
  double t0, t1;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;  // Ray misses bounding box - skip entire subtree
  }

  // Check if the intersection is within the ray's valid range
  if (t1 < ray.min_t || t0 > ray.max_t) {
    return false;
  }

  // If this is a leaf node - test primitives
  if (node->isLeaf()) {
    //cout << "Testing LEAF with " << (node->end - node->start) << " primitives" << endl;
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->intersect(ray, i)) {
        hit = true;
      }
    }
    return hit;
  }
  else {
    // Internal node - recursively test both children
    bool hit_left = false;
    bool hit_right = false;

    if (node->l) {
      hit_left = intersect(ray, i, node->l);
    }
    if (node->r) {
      hit_right = intersect(ray, i, node->r);
    }

    return hit_left || hit_right;
  }
}

} // namespace SceneObjects
} // namespace CGL
