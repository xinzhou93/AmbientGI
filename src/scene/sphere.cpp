#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  Vector3D d = r.d;
  Vector3D o_ray = r.o;
  Vector3D o_sphere = this->o;

  Vector3D camera_to_sphere = o_sphere - o_ray;

  // Quadratic formula setup
  double a = dot(d, d);
  Vector3D oc = o_ray - o_sphere;
  double b = 2.0 * dot(oc, d);
  double c = dot(oc, oc) - this->r2;

  double discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    return false;  // No intersection
  }

  // Calculate both solutions
  t1 = (-b - sqrt(discriminant)) / (2 * a);
  t2 = (-b + sqrt(discriminant)) / (2 * a);

  // Ensure t1 <= t2
  if (t1 > t2) {
    double temp = t1; t1 = t2; t2 = temp;
  }

  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2;
  if (!test(r, t1, t2)) return false;
  if ((t1 < r.min_t) && (t2 < r.min_t)) return false;
  if ((t1 > r.max_t) && (t2 > r.max_t)) return false;

  double t = 0.0f;
  if (t1 >= r.min_t && t1 <= r.max_t) {
    t = t1;  // Use closer intersection
  }
  else if (t2 >= r.min_t && t2 <= r.max_t) {
    t = t2;  // Use farther intersection (ray inside sphere)
  }
  else {
    return false;  // No valid intersection
  }

  r.max_t = t;

  return true;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
	Vector3D d = r.d;
	Vector3D o_ray = r.o;
  Vector3D o_sphere = this->o;

  double t1 = 0.0f;
  double t2 = 0.0f;

  if (!test(r, t1, t2)) {
    return false;
  }
  if ((t1 < r.min_t) && (t2 < r.min_t)) return false;
  if ((t1 > r.max_t) && (t2 > r.max_t)) return false;

  double t = 0.0f;
  if (t1 >= r.min_t && t1 <= r.max_t) {
    t = t1;  // Use closer intersection
  }
  else if (t2 >= r.min_t && t2 <= r.max_t) {
    t = t2;  // Use farther intersection (ray inside sphere)
  }
  else {
    return false;  // No valid intersection
  }

  r.max_t = t;

  // calculate intersection point
  Vector3D pt_intersect = o_ray + t * d;

  // calculate normal vector
  Vector3D N = (pt_intersect - o_sphere).unit();

  // update
  i->t = t;
  i->n = N;
  i->primitive = this;
  i->bsdf = this->get_bsdf();

  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
