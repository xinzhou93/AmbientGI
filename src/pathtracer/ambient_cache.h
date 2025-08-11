#ifndef CGL_AMBIENT_CACHE_H
#define CGL_AMBIENT_CACHE_H

#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include <vector>
#include <unordered_map>

namespace CGL {

// Simple cache entry for storing ambient lighting values
struct AmbientCacheEntry {
    Vector3D position;      // World space position
    Vector3D normal;        // Surface normal
    Vector3D ambient_value; // Cached ambient lighting
    bool is_valid;          // Whether this entry contains valid data
    
    AmbientCacheEntry() : is_valid(false) {}
    
    AmbientCacheEntry(const Vector3D& pos, const Vector3D& norm, const Vector3D& ambient)
        : position(pos), normal(norm), ambient_value(ambient), is_valid(true) {}
};

// Simple grid-based ambient cache
class SimpleAmbientCache {
public:
    SimpleAmbientCache(double grid_spacing = 0.5);
    ~SimpleAmbientCache();
    
    // Try to get cached ambient value through interpolation
    // Returns true if successful, false if need to compute
    bool lookup(const Vector3D& position, const Vector3D& normal, Vector3D& ambient_out);
    
    // Store a new ambient value in the cache
    void store(const Vector3D& position, const Vector3D& normal, const Vector3D& ambient_value);
    
    // Clear all cached values
    void clear();
    
    // Get cache statistics
    int get_num_entries() const { return cache_entries.size(); }
    int get_cache_hits() const { return cache_hits; }
    int get_cache_misses() const { return cache_misses; }
    
    // Get all cache entries for visualization
    const std::vector<AmbientCacheEntry>& get_all_entries() const { return cache_entries; }
    
private:
    double grid_spacing;    // Distance between grid points
    double max_distance;    // Maximum distance for cache lookup
    double normal_threshold; // Minimum dot product for normal compatibility
    
    std::vector<AmbientCacheEntry> cache_entries;
    std::unordered_map<long long, int> grid_to_index; // Maps grid coordinates to cache index
    
    // Statistics
    mutable int cache_hits;
    mutable int cache_misses;
    
    // Helper functions
    long long grid_hash(int gx, int gy, int gz) const;
    void world_to_grid(const Vector3D& world_pos, int& gx, int& gy, int& gz) const;
    Vector3D grid_to_world(int gx, int gy, int gz) const;
    
    // Find nearby cache entries for interpolation
    void find_nearby_entries(const Vector3D& position, const Vector3D& normal, 
                           std::vector<const AmbientCacheEntry*>& nearby_entries) const;
};

} // namespace CGL

#endif // CGL_AMBIENT_CACHE_H