#ifndef CGL_OCTREE_AMBIENT_CACHE_H
#define CGL_OCTREE_AMBIENT_CACHE_H

#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include "scene/bbox.h"
#include <vector>
#include <memory>
#include <functional>

namespace CGL {

// Enhanced cache entry with gradient information
struct GradientCacheEntry {
    Vector3D position;      // World space position
    Vector3D normal;        // Surface normal
    Vector3D ambient_value; // Cached ambient lighting
    
    // Gradient information for better interpolation
    Vector3D gradient_x;    // Gradient in x direction
    Vector3D gradient_y;    // Gradient in y direction
    Vector3D gradient_z;    // Gradient in z direction
    
    double radius;          // Validity radius
    double harmonic_mean;   // Harmonic mean distance for weighting
    bool is_valid;          // Whether this entry contains valid data
    
    GradientCacheEntry() : radius(1.0), harmonic_mean(1.0), is_valid(false) {}
    
    GradientCacheEntry(const Vector3D& pos, const Vector3D& norm, const Vector3D& ambient)
        : position(pos), normal(norm), ambient_value(ambient), 
          radius(1.0), harmonic_mean(1.0), is_valid(true) {}
};

// Octree node for spatial subdivision
class OctreeNode {
public:
    BBox bounds;                                    // Bounding box of this node
    std::unique_ptr<OctreeNode> children[8];       // Child nodes (nullptr if leaf)
    std::vector<GradientCacheEntry*> entries;      // Cache entries (only in leaf nodes)
    
    static const int MAX_ENTRIES_PER_NODE = 8;     // Maximum entries before subdivision
    static const int MAX_DEPTH = 12;                // Maximum octree depth
    
    OctreeNode(const BBox& bbox) : bounds(bbox) {}
    
    // Check if this is a leaf node
    bool is_leaf() const { 
        return children[0] == nullptr; 
    }
    
    // Get child index for a given position
    int get_child_index(const Vector3D& pos) const;
    
    // Create child nodes
    void subdivide();
};

// Octree-based ambient cache with gradient interpolation
class OctreeAmbientCache {
public:
    OctreeAmbientCache(const BBox& scene_bounds, double min_spacing = 0.1);
    ~OctreeAmbientCache();
    
    // Try to get cached ambient value through gradient-based interpolation
    // Returns true if successful, false if need to compute
    bool lookup(const Vector3D& position, const Vector3D& normal, Vector3D& ambient_out);
    
    // Store a new ambient value with gradients in the cache
    void store(const Vector3D& position, const Vector3D& normal, 
               const Vector3D& ambient_value, const Vector3D& grad_x, 
               const Vector3D& grad_y, const Vector3D& grad_z);
    
    // Clear all cached values
    void clear();
    
    // Get cache statistics
    int get_num_entries() const { return total_entries; }
    int get_cache_hits() const { return cache_hits; }
    int get_cache_misses() const { return cache_misses; }
    
    // Get all cache entries for visualization
    void get_all_entries(std::vector<const GradientCacheEntry*>& entries) const;
    
    // Parameters for gradient interpolation
    void set_gradient_threshold(double threshold) { gradient_threshold = threshold; }
    void set_min_spacing(double spacing) { min_spacing = spacing; }
    void set_max_error(double error) { max_error = error; }
    
private:
    std::unique_ptr<OctreeNode> root;
    BBox scene_bounds;
    double min_spacing;        // Minimum distance between cache points
    double gradient_threshold; // Threshold for gradient validity
    double max_error;         // Maximum acceptable error
    double normal_threshold;  // Minimum dot product for normal compatibility
    
    // Statistics
    mutable int cache_hits;
    mutable int cache_misses;
    int total_entries;
    
    // Memory limit
    static const int MAX_CACHE_ENTRIES = 100000;  // Limit total entries to prevent memory issues
    
    // Memory pool for cache entries
    std::vector<std::unique_ptr<GradientCacheEntry>> entry_pool;
    
    // Helper functions
    void insert_entry(OctreeNode* node, GradientCacheEntry* entry, int depth);
    void find_nearby_entries(OctreeNode* node, const Vector3D& position, const Vector3D& normal,
                            std::vector<const GradientCacheEntry*>& nearby_entries) const;
    void collect_all_entries(const OctreeNode* node, std::vector<const GradientCacheEntry*>& entries) const;
    
    // Gradient-based interpolation
    Vector3D interpolate_with_gradients(const std::vector<const GradientCacheEntry*>& entries,
                                       const Vector3D& position, const Vector3D& normal) const;
    
    // Error estimation using gradients
    double estimate_error(const GradientCacheEntry& entry, const Vector3D& position) const;
    
    // Check if a new sample is needed based on existing entries
    bool need_new_sample(const Vector3D& position, const Vector3D& normal,
                        const std::vector<const GradientCacheEntry*>& nearby_entries) const;
};

} // namespace CGL

#endif // CGL_OCTREE_AMBIENT_CACHE_H