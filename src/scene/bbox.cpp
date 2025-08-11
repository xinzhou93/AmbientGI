#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
  
  // Slab method for ray-box intersection
  // X-axis slab
  // Temporary: disable all BBox culling
  //t0 = r.min_t;
  //t1 = r.max_t;
  //return true;

  double tmin = (min.x - r.o.x) / r.d.x;
  double tmax = (max.x - r.o.x) / r.d.x;
  if (tmin > tmax) std::swap(tmin, tmax);

  // Y-axis slab
  double tymin = (min.y - r.o.y) / r.d.y;
  double tymax = (max.y - r.o.y) / r.d.y;
  if (tymin > tymax) std::swap(tymin, tymax);

  // Z-axis slab
  double tzmin = (min.z - r.o.z) / r.d.z;
  double tzmax = (max.z - r.o.z) / r.d.z;
  if (tzmin > tzmax) std::swap(tzmin, tzmax);

  // Find intersection of all three slabs
  double t_enter = std::max({ tmin, tymin, tzmin });
  double t_exit = std::min({ tmax, tymax, tzmax });

  // Check if ray intersects the box
  if (t_enter > t_exit) {
    return false;  // Ray misses the box
  }

  // Update t0 and t1 with the intersection interval
  t0 = t_enter;
  t1 = t_exit;

  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
