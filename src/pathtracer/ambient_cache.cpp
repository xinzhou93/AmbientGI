#include "ambient_cache.h"
#include <cmath>
#include <algorithm>

namespace CGL {

SimpleAmbientCache::SimpleAmbientCache(double grid_spacing) 
    : grid_spacing(grid_spacing), cache_hits(0), cache_misses(0) {
    // Set thresholds based on grid spacing
    max_distance = grid_spacing * 2.0;  // Look within 2 grid cells
    normal_threshold = 0.8;  // Normals must be reasonably similar (cos > 0.8)
}

SimpleAmbientCache::~SimpleAmbientCache() {
    clear();
}

bool SimpleAmbientCache::lookup(const Vector3D& position, const Vector3D& normal, Vector3D& ambient_out) {
    // Find nearby cache entries
    std::vector<const AmbientCacheEntry*> nearby_entries;
    find_nearby_entries(position, normal, nearby_entries);
    
    if (nearby_entries.empty()) {
        cache_misses++;
        return false;  // Cache miss - need to compute
    }
    
    // Simple interpolation using inverse distance weighting
    Vector3D weighted_sum(0, 0, 0);
    double total_weight = 0.0;
    
    for (const AmbientCacheEntry* entry : nearby_entries) {
        double distance = (entry->position - position).norm();
        if (distance < 1e-6) distance = 1e-6;  // Avoid division by zero
        
        double weight = 1.0 / (distance * distance);  // Inverse square distance
        
        // Weight by normal similarity
        double normal_similarity = dot(entry->normal, normal);
        if (normal_similarity > normal_threshold) {
            weight *= normal_similarity;
            weighted_sum += entry->ambient_value * weight;
            total_weight += weight;
        }
    }
    
    if (total_weight > 1e-6) {
        ambient_out = weighted_sum / total_weight;
        cache_hits++;
        return true;  // Cache hit!
    }
    
    cache_misses++;
    return false;  // No valid entries found
}

void SimpleAmbientCache::store(const Vector3D& position, const Vector3D& normal, const Vector3D& ambient_value) {
    // Convert to grid coordinates
    int gx, gy, gz;
    world_to_grid(position, gx, gy, gz);
    
    long long hash = grid_hash(gx, gy, gz);
    
    // Check if we already have an entry at this grid location
    auto it = grid_to_index.find(hash);
    if (it != grid_to_index.end()) {
        // Update existing entry
        cache_entries[it->second] = AmbientCacheEntry(position, normal, ambient_value);
    } else {
        // Add new entry
        cache_entries.push_back(AmbientCacheEntry(position, normal, ambient_value));
        grid_to_index[hash] = cache_entries.size() - 1;
    }
}

void SimpleAmbientCache::clear() {
    cache_entries.clear();
    grid_to_index.clear();
    cache_hits = 0;
    cache_misses = 0;
}

long long SimpleAmbientCache::grid_hash(int gx, int gy, int gz) const {
    // Simple hash function for 3D grid coordinates
    return ((long long)gx << 20) | ((long long)gy << 10) | ((long long)gz);
}

void SimpleAmbientCache::world_to_grid(const Vector3D& world_pos, int& gx, int& gy, int& gz) const {
    gx = (int)floor(world_pos.x / grid_spacing);
    gy = (int)floor(world_pos.y / grid_spacing);
    gz = (int)floor(world_pos.z / grid_spacing);
}

Vector3D SimpleAmbientCache::grid_to_world(int gx, int gy, int gz) const {
    return Vector3D(gx * grid_spacing, gy * grid_spacing, gz * grid_spacing);
}

void SimpleAmbientCache::find_nearby_entries(const Vector3D& position, const Vector3D& normal, 
                                           std::vector<const AmbientCacheEntry*>& nearby_entries) const {
    nearby_entries.clear();
    
    // Search in a small radius around the position
    int search_radius = 2;  // Check 2 grid cells in each direction
    
    int center_gx, center_gy, center_gz;
    world_to_grid(position, center_gx, center_gy, center_gz);
    
    for (int dx = -search_radius; dx <= search_radius; dx++) {
        for (int dy = -search_radius; dy <= search_radius; dy++) {
            for (int dz = -search_radius; dz <= search_radius; dz++) {
                int gx = center_gx + dx;
                int gy = center_gy + dy;
                int gz = center_gz + dz;
                
                long long hash = grid_hash(gx, gy, gz);
                auto it = grid_to_index.find(hash);
                
                if (it != grid_to_index.end()) {
                    const AmbientCacheEntry& entry = cache_entries[it->second];
                    if (entry.is_valid) {
                        double distance = (entry.position - position).norm();
                        if (distance <= max_distance) {
                            nearby_entries.push_back(&entry);
                        }
                    }
                }
            }
        }
    }
}

} // namespace CGL