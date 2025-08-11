#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"

using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();
  ambient_cache = new SimpleAmbientCache(0.5); // 0.5 unit grid spacing default
  octree_cache = nullptr; // Will be initialized when scene bounds are known

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
  
  use_ambient_gi = false;
  visualize_cache = false;
  use_octree_cache = true; // Re-enable octree cache with safety fixes
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
  delete ambient_cache;
  delete octree_cache;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
  
  // Clear ambient caches
  if (ambient_cache) {
    ambient_cache->clear();
  }
  if (octree_cache) {
    octree_cache->clear();
  }
}

void PathTracer::init_octree_cache() {
  // Initialize octree cache with scene bounds when BVH is available
  if (bvh && use_octree_cache && !octree_cache) {
    BBox scene_bounds = bvh->get_bbox();
    // Expand bounds slightly to ensure all points are inside
    Vector3D expand(0.1, 0.1, 0.1);
    scene_bounds.min -= expand;
    scene_bounds.max += expand;
    
    // Create octree cache with appropriate spacing
    double min_spacing = 0.015; // Reduced for denser cache and smoother results
    
    try {
      octree_cache = new OctreeAmbientCache(scene_bounds, min_spacing);
      
      // Configure octree parameters
      octree_cache->set_gradient_threshold(0.1);
      octree_cache->set_max_error(0.05);
      
      printf("[PathTracer] Initialized octree ambient cache with bounds: (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)\n",
             scene_bounds.min.x, scene_bounds.min.y, scene_bounds.min.z,
             scene_bounds.max.x, scene_bounds.max.y, scene_bounds.max.z);
    } catch (const std::exception& e) {
      printf("[PathTracer] Error initializing octree cache: %s\n", e.what());
      octree_cache = nullptr;
      use_octree_cache = false;
    }
  }
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, 
  // you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, 
  // however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  double pdf = 1.0 / (2.0 * PI);
  int light_hits = 0;

  for (int i = 0; i < num_samples; i++) {
    // initialize intersection
    Intersection isect2;
    // get a uniform sample in the object space
    Vector3D wi = hemisphereSampler->get_sample();
    // transform it to world space
    wi = o2w * wi;
    // create shadow ray in the world space
    Ray secondRay(hit_p, wi);
    secondRay.min_t = EPS_F;

    // check if the shadow ray intsects with sth
    if (!bvh->intersect(secondRay, &isect2)) {
      continue;
    }
    else {
      //check if this sample direction has intersection with the light source
      Vector3D result = isect2.bsdf->get_emission();
      if (!(result.x == 0 && result.y == 0 && result.z == 0)) { // hit the light source
        light_hits++;
        // evaluate reflection
        Vector3D wio = w2o * wi;
        Vector3D reflection = isect.bsdf->f(w_out, wio);
        L_out += (reflection * result * cos_theta(wio)) / pdf;
      }
      else continue;
    }
  }

  //std::cout << "Light hits: " << light_hits << " out of " << num_samples << std::endl;
  L_out = (1.0 / num_samples) * L_out;
  return L_out;

  //Simple test: return surface normal as color
  //return (isect.n + Vector3D(1, 1, 1)) * 0.5;  // Normal mapping for debug
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  // Iterate over all lights in the scene
  for (auto& light : scene->lights) {
    if (light->is_delta_light()) {
      // Delta lights (point lights, directional lights) - sample once
      Vector3D wi;
      double distToLight;
      double pdf;
      
      Vector3D radiance = light->sample_L(hit_p, &wi, &distToLight, &pdf);

      if (pdf > 0) {
        // Create shadow ray
        Ray shadowRay(hit_p, wi);
        shadowRay.min_t = EPS_F;
        shadowRay.max_t = distToLight - EPS_F;

        // Transform wi to local coordinates
        Vector3D wi_local = w2o * wi;
        
        // Check if light is above the surface
        if (wi_local.z > 0) {
          // Check for shadows
          if (!bvh->has_intersection(shadowRay)) {
           // Evaluate BSDF
           Vector3D f = isect.bsdf->f(w_out, wi_local);
           // Add contribution: radiance * f * cos(theta) / pdf
           L_out += radiance * f * wi_local.z / pdf;
          }
        }
      }
    } else {
      // Area lights - sample multiple times
      Vector3D area_light_contribution;
      
      for (int s = 0; s < ns_area_light; s++) {
        Vector3D wi;
        double distToLight;
        double pdf;
        
        Vector3D radiance = light->sample_L(hit_p, &wi, &distToLight, &pdf);

        if (pdf > 0) {
          // Create shadow ray
          Ray shadowRay(hit_p, wi);
          shadowRay.min_t = EPS_F;
          shadowRay.max_t = distToLight - EPS_F;

          // Transform wi to local coordinates
          Vector3D wi_local = w2o * wi;
          
          // Check if light is above the surface
          if (wi_local.z > 0) {
            // Check for shadows
            if (!bvh->has_intersection(shadowRay)) {
              // Evaluate BSDF
              Vector3D f = isect.bsdf->f(w_out, wi_local);
              
              // Add contribution: radiance * f * cos(theta) / pdf
              area_light_contribution += radiance * f * wi_local.z / pdf;
            }
          }
        }
      }
      
      // Average the area light samples
      L_out += area_light_contribution / ns_area_light;
    }
  }
  
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  Vector3D result = isect.bsdf->get_emission();
  return result;
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  }
  else {
    return estimate_direct_lighting_importance(r, isect);
  }
}

Vector3D PathTracer::compute_ambient_lighting(const Ray &r, const Intersection &isect) {
  // Stable hemisphere sampling for ambient cache computation
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // Use hemisphere sampling with good balance of quality vs performance
  int num_samples = 512;
  Vector3D L_out;
  double pdf = 1.0 / (2.0 * PI);

  for (int i = 0; i < num_samples; i++) {
    Intersection isect2;
    Vector3D wi = hemisphereSampler->get_sample();
    wi = o2w * wi;
    Ray secondRay(hit_p, wi);
    secondRay.min_t = EPS_F;

    if (bvh->intersect(secondRay, &isect2)) {
      // Get lighting: emission + direct lighting (zero + one bounce)
      Vector3D incoming_light = zero_bounce_radiance(secondRay, isect2) + 
                               one_bounce_radiance(secondRay, isect2);
      
      // Apply BRDF
      Vector3D wio = w2o * wi;
      Vector3D reflection = isect.bsdf->f(w_out, wio);
      L_out += (reflection * incoming_light * cos_theta(wio)) / pdf;
    }
  }

  return L_out / num_samples;
}

// Helper function not currently used - reverted to stable hemisphere sampling
/*
Vector3D PathTracer::compute_ambient_lighting_helper(const Ray& r, const Intersection& isect, int depth) {
  // Recursive Monte Carlo implementation with proper depth tracking
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();
  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);
  Vector3D L_out(0, 0, 0);
  Vector3D w_in(0, 0, 0);
  double pdf = 0.0f;
  Intersection isect2;

  if (isAccumBounces) {
    // accumuate direct lighting
    if (max_ray_depth > 0) {
      L_out = one_bounce_radiance(r, isect);
    }

    // Only do indirect bounces if max_ray_depth > 1
    if (max_ray_depth > 1) {
      double termination_prob = 0.35;
      double continue_prob = 1 - termination_prob;
      bool isContinue = true;

      if (depth > 1) {
        // Russian Roulette termination check after first indirect bounce
        isContinue = !coin_flip(termination_prob);
      }

      if (isContinue) {
        // sample BSDF at intsection of the hit point
        Vector3D rad = isect.bsdf->sample_f(w_out, &w_in, &pdf);

        // Create a shadow ray
        Vector3D wi_w = o2w * w_in;
        if (w_in.z <= 0) return L_out;

        Ray shadowRay(hit_p, wi_w);
        shadowRay.min_t = EPS_F;
        shadowRay.depth = r.depth;

        // check if shadow ray has intersectio with the next object
        // it can be light source or another scene object
        if (pdf > 0.0f) {
          if (bvh->intersect(shadowRay, &isect2)) {
            if (depth > 1) {
              // Use Russian Roulette probability division
              L_out += compute_ambient_lighting_helper(shadowRay, isect2, depth + 1) * rad * cos_theta(w_in) / (pdf * continue_prob);
            }
            else {
              // First indirect bounce - no probability division
              L_out += compute_ambient_lighting_helper(shadowRay, isect2, depth + 1) * rad * cos_theta(w_in) / pdf;
            }
          }
        }
      }
    }
  }
  return L_out;
}
*/

Vector3D PathTracer::compute_ambient_lighting_cached(const Ray &r, const Intersection &isect) {

  const Vector3D hit_p = r.o + r.d * isect.t;
  
  // Try to get cached ambient value
  Vector3D cached_ambient;
  
  if (use_octree_cache && octree_cache) {
    // Use octree cache with gradient interpolation
    if (octree_cache->lookup(hit_p, isect.n, cached_ambient)) {
      return cached_ambient;
    }
    // Cache miss - compute ambient lighting with gradients
    Vector3D computed_ambient = compute_ambient_lighting(r, isect);
    
    // For now, use zero gradients to avoid infinite recursion
    // TODO: Implement proper gradient computation without recursion
    Vector3D grad_x(0, 0, 0);
    Vector3D grad_y(0, 0, 0);
    Vector3D grad_z(0, 0, 0);
    
    // Store with gradients
    octree_cache->store(hit_p, isect.n, computed_ambient, grad_x, grad_y, grad_z);
    return computed_ambient;
  } else {
    // Use simple grid cache
    if (ambient_cache->lookup(hit_p, isect.n, cached_ambient)) {
      return cached_ambient;
    }
    // Cache miss - compute and store
    Vector3D computed_ambient = compute_ambient_lighting(r, isect);
    ambient_cache->store(hit_p, isect.n, computed_ambient);
    return computed_ambient;
  }
}

bool PathTracer::intersect_cache_visualization(const Ray& r, double& t, Vector3D& color) {
  if (!visualize_cache || !use_ambient_gi) {
    return false;
  }
  
  const double sphere_radius = 0.05; // Small spheres for cache points
  double closest_t = INF_D;
  Vector3D closest_color;
  bool hit = false;
  
  // Check intersection with each cache point
  if (use_octree_cache && octree_cache) {
    // Get entries from octree cache
    std::vector<const GradientCacheEntry*> octree_entries;
    octree_cache->get_all_entries(octree_entries);
    
    for (const auto* entry : octree_entries) {
      if (!entry->is_valid) continue;
      
      // Ray-sphere intersection
      Vector3D oc = r.o - entry->position;
      double a = dot(r.d, r.d);
      double b = 2.0 * dot(oc, r.d);
      double c = dot(oc, oc) - sphere_radius * sphere_radius;
      double discriminant = b * b - 4 * a * c;
      
      if (discriminant >= 0) {
        double t1 = (-b - sqrt(discriminant)) / (2.0 * a);
        double t2 = (-b + sqrt(discriminant)) / (2.0 * a);
        double sphere_t = (t1 > EPS_F) ? t1 : t2;
        
        if (sphere_t > EPS_F && sphere_t < closest_t) {
          closest_t = sphere_t;
          // Color cache points by their ambient value intensity
          double intensity = entry->ambient_value.illum();
          intensity = clamp(intensity, 0.0, 1.0);
          // Color gradient: blue (low) -> green (medium) -> red (high)
          if (intensity < 0.5) {
            closest_color = Vector3D(0, intensity * 2, 1 - intensity * 2);
          } else {
            closest_color = Vector3D((intensity - 0.5) * 2, 1 - (intensity - 0.5) * 2, 0);
          }
          hit = true;
        }
      }
    }
  } else {
    // Use simple cache entries
    const auto& cache_entries = ambient_cache->get_all_entries();
    for (const auto& entry : cache_entries) {
      if (!entry.is_valid) continue;
    
    // Ray-sphere intersection
    Vector3D oc = r.o - entry.position;
    double a = dot(r.d, r.d);
    double b = 2.0 * dot(oc, r.d);
    double c = dot(oc, oc) - sphere_radius * sphere_radius;
    double discriminant = b * b - 4 * a * c;
    
    if (discriminant >= 0) {
      double t1 = (-b - sqrt(discriminant)) / (2.0 * a);
      double t2 = (-b + sqrt(discriminant)) / (2.0 * a);
      double sphere_t = (t1 > EPS_F) ? t1 : t2;
      
      if (sphere_t > EPS_F && sphere_t < closest_t) {
        closest_t = sphere_t;
        // Color cache points by their ambient value intensity
        double intensity = entry.ambient_value.illum();
        intensity = clamp(intensity, 0.0, 1.0);
        // Color gradient: blue (low) -> green (medium) -> red (high)
        if (intensity < 0.5) {
          closest_color = Vector3D(0, intensity * 2, 1 - intensity * 2);
        } else {
          closest_color = Vector3D((intensity - 0.5) * 2, 1 - (intensity - 0.5) * 2, 0);
        }
        hit = true;
      }
    }
    }
  }
  
  if (hit) {
    t = closest_t;
    color = closest_color;
  }
  
  return hit;
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {

  //START of the final project
  if (use_ambient_gi) {
    // Use cached ambient computation with interpolation
    return compute_ambient_lighting_cached(r, isect);
  }
  
  // use the recursive Monte Carlo method as the fallback (PART 4)
  int depth = 0;
  return at_least_one_bounce_radiance_helper(r, isect, depth);
}

Vector3D PathTracer::at_least_one_bounce_radiance_helper(const Ray& r, const Intersection& isect, int depth) {
  //std::cout << "Depth: " << depth << ", Max depth: " << r.depth << std::endl;
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();
  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);
  Vector3D L_out(0, 0, 0);
  Vector3D w_in(0, 0, 0);
  double pdf = 0.0f;
  Intersection isect2;

  // base case
  // PART4 TASK2
  //if (depth >= r.depth) {
  //  //std::cout << "Reached max depth, returning black" << std::endl;
  //  return L_out;
  //}

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  //if (depth == r.depth - 1) {
  //  L_out = one_bounce_radiance(r, isect);
  //  //std::cout << "Added direct lighting: " << L_out.x << " " << L_out.y << " " << L_out.z << std::endl;
  //}
  
  // TASK2
  // base case
  //if (depth >= r.depth) {
  //  return L_out;
  //}


  //// Add direct lighting at every intersection in accumulating mode
  //if (isAccumBounces) {
  //  if (r.depth > 0) {
  //    L_out = one_bounce_radiance(r, isect);
  //  }
  //} else {
  //  // Non-accumulating: only add direct at final depth
  //  if (depth == r.depth - 1 && r.depth > 0) {
  //    L_out = one_bounce_radiance(r, isect);
  //  }
  //}

  //Vector3D rad = isect.bsdf->sample_f(w_out, &w_in, &pdf);

  //if (pdf > 0.0f) {
  //  if (w_in.z <= 0) return L_out;

  //  Vector3D wi_w = o2w * w_in;
  //  Ray shadowRay(hit_p, wi_w);
  //  shadowRay.min_t = EPS_F;
  //  //update depth or it will be 0
  //  shadowRay.depth = r.depth;

  //  if (bvh->intersect(shadowRay, &isect2)) {
  //    L_out += at_least_one_bounce_radiance_helper(shadowRay, isect2, depth + 1) * rad * cos_theta(w_in) / pdf;
  //  }
  //}
  //return L_out;

  if (isAccumBounces) {
    // accumuate direct lighting
    if (r.depth > 0) {
      L_out = one_bounce_radiance(r, isect);
    }

    // Only do indirect bounces if max_ray_depth > 1
    if (r.depth > 1) {
      double termination_prob = 0.35;
      double continue_prob = 1 - termination_prob;
      bool isContinue = true;

      if (depth > 1) {
        // Russian Roulette termination check after first indirect bounce
        isContinue = !coin_flip(termination_prob);
      }

      if (isContinue) {
      // sample BSDF at intsection of the hit point
      Vector3D rad = isect.bsdf->sample_f(w_out, &w_in, &pdf);

      // Create a shadow ray
      Vector3D wi_w = o2w * w_in;
      if (w_in.z <= 0) return L_out;

      Ray shadowRay(hit_p, wi_w);
      shadowRay.min_t = EPS_F;
      shadowRay.depth = r.depth;

      // check if shadow ray has intersectio with the next object
      // it can be light source or another scene object
      if (pdf > 0.0f) {
        if (bvh->intersect(shadowRay, &isect2)) {
          if (depth > 1) {
            // Use Russian Roulette probability division
            L_out += at_least_one_bounce_radiance_helper(shadowRay, isect2, depth + 1) * rad * cos_theta(w_in) / (pdf * continue_prob);
          }
          else {
            // First indirect bounce - no probability division
            L_out += at_least_one_bounce_radiance_helper(shadowRay, isect2, depth + 1) * rad * cos_theta(w_in) / pdf;
          }
        }
      }
      } 
    }  
  }
  else {
    // base case
    if (depth >= r.depth) {
      return L_out;
    }

    if (depth == r.depth - 1 && r.depth > 0) {
      L_out = one_bounce_radiance(r, isect);
     }

    Vector3D rad = isect.bsdf->sample_f(w_out, &w_in, &pdf);

    if (pdf > 0.0f) {
      if (w_in.z <= 0) return L_out;

      Vector3D wi_w = o2w * w_in;
      Ray shadowRay(hit_p, wi_w);
      shadowRay.min_t = EPS_F;
      //update depth or it will be 0
      shadowRay.depth = r.depth;

      if (bvh->intersect(shadowRay, &isect2)) {
        L_out += at_least_one_bounce_radiance_helper(shadowRay, isect2, depth + 1) * rad * cos_theta(w_in) / pdf;
      }
    }
  }
  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // Check for cache visualization first (before scene intersection)
  double viz_t;
  Vector3D viz_color;
  if (intersect_cache_visualization(r, viz_t, viz_color)) {
    // Check if cache sphere is closer than scene geometry
    bool scene_hit = bvh->intersect(r, &isect);
    if (!scene_hit || viz_t < isect.t) {
      return viz_color; // Render cache point
    }
  }

  // Normal scene intersection
  if (!bvh->intersect(r, &isect)) {
    return Vector3D(0, 0, 0);
  }
  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
  //return envLight ? envLight->sample_dir(r) : L_out;

  // TODO (Part 3): Return the direct illumination.
  //return zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);
  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  // SPECIAL MODE: Accumulate indirect bounces
  //bool accumulate_indirect_only = true; 
  //
  //if (accumulate_indirect_only) {
  //  // Accumulate bounces 1 through max_ray_depth
  //  Vector3D indirect_sum(0, 0, 0);
  //  for (int bounce = 1; bounce <= max_ray_depth; bounce++) {
  //    Ray temp_ray = r;
  //    temp_ray.depth = bounce;
  //    // Use non-accumulating mode to get single bounce
  //    indirect_sum += at_least_one_bounce_radiance(temp_ray, isect);
  //  }
  //  return indirect_sum;
  //}
  
  if (isAccumBounces) {
    // Accumulate all bounces (normal global illumination)
    if (use_ambient_gi) {
      // Add direct lighting + ambient GI
      return zero_bounce_radiance(r, isect) + 
             one_bounce_radiance(r, isect) + 
             compute_ambient_lighting_cached(r, isect);
    } else {
      return zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
    }
  }
  else {
    // Show only the bounce level specified by max_ray_depth
    if (max_ray_depth == 0) {
      return zero_bounce_radiance(r, isect);  // Only emission/lights
    }
    else {
      return at_least_one_bounce_radiance(r, isect);  // Only bounced light
    }
  }
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
  int max_samples = ns_aa;       // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D rad_sum (0,0,0);
  double illum_sum = 0.0;
  double illum_sum_squared = 0.0;

  int samples_taken = 0;

  while (samples_taken < max_samples) {
    //batch sample
    int batch_size = samplesPerBatch;
    if (batch_size > max_samples - samples_taken) {
      batch_size = max_samples - samples_taken;
    }

    for (int i{ 0 }; i < batch_size; ++i) {
      // 1. get random samples in a pixel
      Vector2D sample = gridSampler->get_sample();
      // 2. generate ray with random sample
      double x_norm = (origin.x + sample.x) / sampleBuffer.w;
      double y_norm = (origin.y + sample.y) / sampleBuffer.h;
      // 3. create a random ray inside this pixel
      Ray ray = camera->generate_ray(x_norm, y_norm);
      // initialize camera rays depth
      ray.depth = max_ray_depth;
      //std::cout << "Ray created: max_t=" << ray.max_t << std::endl;
      // 4. get the radiance of this pixel
      Vector3D radiance = est_radiance_global_illumination(ray);
      rad_sum += radiance;
      
      // Use illum() for variance calculation
      double illum = radiance.illum();
      illum_sum += illum;
      illum_sum_squared += illum * illum;
      
      samples_taken++;
    }
    // check for convergence for every batch samples
    if (samples_taken >= samplesPerBatch) {
      // Calculate mean and variance using illuminance
      double mean_illum = illum_sum / samples_taken;
      double variance = (illum_sum_squared - illum_sum * illum_sum / samples_taken) / (samples_taken - 1);

      // Convergence test
      double I = 1.96 * sqrt(variance / samples_taken);

      //stop sampling if converged
      if (I <= maxTolerance * mean_illum) {
        break;
      }
    }
  }

  Vector3D avg_radiance = rad_sum / samples_taken;
  sampleBuffer.update_pixel(avg_radiance, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = samples_taken;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
