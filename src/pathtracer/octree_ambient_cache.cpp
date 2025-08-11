#include "octree_ambient_cache.h"
#include <cmath>
#include <algorithm>

namespace CGL {

// OctreeNode implementation

int OctreeNode::get_child_index(const Vector3D& pos) const {
    Vector3D center = bounds.centroid();
    int index = 0;
    if (pos.x > center.x) index |= 1;
    if (pos.y > center.y) index |= 2;
    if (pos.z > center.z) index |= 4;
    return index;
}

void OctreeNode::subdivide() {
    Vector3D min = bounds.min;
    Vector3D max = bounds.max;
    Vector3D center = bounds.centroid();
    
    // Check if bounds are valid
    if (min.x >= max.x || min.y >= max.y || min.z >= max.z) {
        return; // Invalid bounds, don't subdivide
    }
    
    // Create 8 child nodes
    for (int i = 0; i < 8; i++) {
        Vector3D child_min, child_max;
        
        child_min.x = (i & 1) ? center.x : min.x;
        child_min.y = (i & 2) ? center.y : min.y;
        child_min.z = (i & 4) ? center.z : min.z;
        
        child_max.x = (i & 1) ? max.x : center.x;
        child_max.y = (i & 2) ? max.y : center.y;
        child_max.z = (i & 4) ? max.z : center.z;
        
        children[i] = std::make_unique<OctreeNode>(BBox(child_min, child_max));
    }
    
    // Move entries to children
    for (auto* entry : entries) {
        if (entry) { // Safety check
            int child_idx = get_child_index(entry->position);
            if (child_idx >= 0 && child_idx < 8 && children[child_idx]) {
                children[child_idx]->entries.push_back(entry);
            }
        }
    }
    entries.clear();
}

// OctreeAmbientCache implementation

OctreeAmbientCache::OctreeAmbientCache(const BBox& scene_bounds, double min_spacing)
    : scene_bounds(scene_bounds), min_spacing(min_spacing),
      gradient_threshold(0.1), max_error(0.1), normal_threshold(0.8),
      cache_hits(0), cache_misses(0), total_entries(0) {
    
    root = std::make_unique<OctreeNode>(scene_bounds);
}

OctreeAmbientCache::~OctreeAmbientCache() {
    clear();
}

bool OctreeAmbientCache::lookup(const Vector3D& position, const Vector3D& normal, Vector3D& ambient_out) {
    // Find nearby cache entries
    std::vector<const GradientCacheEntry*> nearby_entries;
    find_nearby_entries(root.get(), position, normal, nearby_entries);
    
    if (nearby_entries.empty()) {
        cache_misses++;
        return false;
    }
    
    // Check if we need a new sample based on gradient error estimation
    if (need_new_sample(position, normal, nearby_entries)) {
        cache_misses++;
        return false;
    }
    
    // Perform gradient-based interpolation
    ambient_out = interpolate_with_gradients(nearby_entries, position, normal);
    cache_hits++;
    return true;
}

void OctreeAmbientCache::store(const Vector3D& position, const Vector3D& normal,
                               const Vector3D& ambient_value, const Vector3D& grad_x,
                               const Vector3D& grad_y, const Vector3D& grad_z) {
    
    // Check if position is within scene bounds (with small tolerance)
    Vector3D tolerance(0.1, 0.1, 0.1);
    BBox expanded_bounds(scene_bounds.min - tolerance, scene_bounds.max + tolerance);
    if (position.x < expanded_bounds.min.x || position.x > expanded_bounds.max.x ||
        position.y < expanded_bounds.min.y || position.y > expanded_bounds.max.y ||
        position.z < expanded_bounds.min.z || position.z > expanded_bounds.max.z) {
        // Position outside bounds, skip
        return;
    }
    
    // Create new cache entry
    auto new_entry = std::make_unique<GradientCacheEntry>(position, normal, ambient_value);
    new_entry->gradient_x = grad_x;
    new_entry->gradient_y = grad_y;
    new_entry->gradient_z = grad_z;
    
    // Compute validity radius based on gradient magnitude
    double grad_mag = std::max({grad_x.norm(), grad_y.norm(), grad_z.norm()});
    new_entry->radius = (grad_mag > 1e-6) ? (max_error / grad_mag) : min_spacing * 4.0;
    new_entry->radius = std::max(min_spacing, std::min(new_entry->radius, min_spacing * 4.0));
    
    // Check memory limit
    if (total_entries >= MAX_CACHE_ENTRIES) {
        return; // Don't add more entries if we're at the limit
    }
    
    // Store in octree
    GradientCacheEntry* entry_ptr = new_entry.get();
    entry_pool.push_back(std::move(new_entry));
    insert_entry(root.get(), entry_ptr, 0);
    total_entries++;
}


void OctreeAmbientCache::clear() {
    root = std::make_unique<OctreeNode>(scene_bounds);
    entry_pool.clear();
    cache_hits = 0;
    cache_misses = 0;
    total_entries = 0;
}

void OctreeAmbientCache::get_all_entries(std::vector<const GradientCacheEntry*>& entries) const {
    entries.clear();
    collect_all_entries(root.get(), entries);
}

void OctreeAmbientCache::insert_entry(OctreeNode* node, GradientCacheEntry* entry, int depth) {
    if (!node || !entry) return;  // Safety check
    
    if (!node->is_leaf()) {
        // Non-leaf node: insert into appropriate child
        int child_idx = node->get_child_index(entry->position);
        if (child_idx >= 0 && child_idx < 8 && node->children[child_idx]) {
            insert_entry(node->children[child_idx].get(), entry, depth + 1);
        }
        return;
    }
    
    // Leaf node: add entry
    node->entries.push_back(entry);
    
    // Check if we need to subdivide
    if (node->entries.size() > OctreeNode::MAX_ENTRIES_PER_NODE && 
        depth < OctreeNode::MAX_DEPTH) {
        node->subdivide();
    }
}

void OctreeAmbientCache::find_nearby_entries(OctreeNode* node, const Vector3D& position, 
                                            const Vector3D& normal,
                                            std::vector<const GradientCacheEntry*>& nearby_entries) const {
    if (!node) return;
    
    // Check if position is within node bounds (with some margin for radius)
    BBox search_box(position - Vector3D(min_spacing * 6), position + Vector3D(min_spacing * 6));
    
    // Check if bounding boxes overlap
    bool overlap = (node->bounds.min.x <= search_box.max.x && node->bounds.max.x >= search_box.min.x) &&
                   (node->bounds.min.y <= search_box.max.y && node->bounds.max.y >= search_box.min.y) &&
                   (node->bounds.min.z <= search_box.max.z && node->bounds.max.z >= search_box.min.z);
    
    if (!overlap) return;
    
    if (node->is_leaf()) {
        // Check all entries in leaf node
        for (const auto* entry : node->entries) {
            if (!entry->is_valid) continue;
            
            double distance = (entry->position - position).norm();
            if (distance <= entry->radius) {
                // Check normal compatibility
                double normal_dot = dot(entry->normal, normal);
                if (normal_dot >= normal_threshold) {
                    nearby_entries.push_back(entry);
                }
            }
        }
    } else {
        // Recursively search children
        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                find_nearby_entries(node->children[i].get(), position, normal, nearby_entries);
            }
        }
    }
}

void OctreeAmbientCache::collect_all_entries(const OctreeNode* node, 
                                           std::vector<const GradientCacheEntry*>& entries) const {
    if (!node) return;
    
    if (node->is_leaf()) {
        for (const auto* entry : node->entries) {
            if (entry->is_valid) {
                entries.push_back(entry);
            }
        }
    } else {
        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                collect_all_entries(node->children[i].get(), entries);
            }
        }
    }
}

Vector3D OctreeAmbientCache::interpolate_with_gradients(
    const std::vector<const GradientCacheEntry*>& entries,
    const Vector3D& position, const Vector3D& normal) const {
    
    if (entries.empty()) return Vector3D(0, 0, 0);
    
    Vector3D weighted_sum(0, 0, 0);
    double total_weight = 0.0;
    
    for (const auto* entry : entries) {
        Vector3D delta = position - entry->position;
        double distance = delta.norm();
        
        // Ward's gradient-based extrapolation
        Vector3D extrapolated = entry->ambient_value + 
                               entry->gradient_x * delta.x +
                               entry->gradient_y * delta.y +
                               entry->gradient_z * delta.z;
        
        // Weight based on distance and normal similarity
        double dist_weight = 1.0 / (distance + 1e-6);
        double normal_weight = dot(entry->normal, normal);
        double weight = dist_weight * normal_weight;
        
        // Additional weight based on gradient error
        double error = estimate_error(*entry, position);
        if (error > 0) {
            weight *= exp(-error / max_error);
        }
        
        weighted_sum += extrapolated * weight;
        total_weight += weight;
    }
    
    return (total_weight > 1e-6) ? (weighted_sum / total_weight) : Vector3D(0, 0, 0);
}

double OctreeAmbientCache::estimate_error(const GradientCacheEntry& entry, 
                                        const Vector3D& position) const {
    Vector3D delta = position - entry.position;
    
    // Estimate error based on second-order Taylor expansion
    // Error ≈ 0.5 * |∇²f| * |delta|²
    // We approximate |∇²f| using gradient magnitude
    double grad_mag = std::max({entry.gradient_x.norm(), 
                               entry.gradient_y.norm(), 
                               entry.gradient_z.norm()});
    
    return 0.5 * grad_mag * delta.norm2();
}

bool OctreeAmbientCache::need_new_sample(const Vector3D& position, const Vector3D& normal,
                                       const std::vector<const GradientCacheEntry*>& nearby_entries) const {
    
    if (nearby_entries.empty()) return true;
    
    // Check if all nearby entries have acceptable error
    for (const auto* entry : nearby_entries) {
        double error = estimate_error(*entry, position);
        if (error > max_error) {
            return true; // Error too high, need new sample
        }
        
        // Also check normal deviation
        double normal_dot = dot(entry->normal, normal);
        if (normal_dot < normal_threshold) {
            return true; // Normal too different
        }
    }
    
    // Check minimum spacing constraint
    for (const auto* entry : nearby_entries) {
        double distance = (entry->position - position).norm();
        if (distance < min_spacing) {
            return false; // Too close to existing sample
        }
    }
    
    return false; // Existing samples are sufficient
}

} // namespace CGL