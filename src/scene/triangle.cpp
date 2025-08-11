#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"
#include <iostream>

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
	p1 = mesh->positions[v1];
	p2 = mesh->positions[v2];
	p3 = mesh->positions[v3];
	n1 = mesh->normals[v1];
	n2 = mesh->normals[v2];
	n3 = mesh->normals[v3];
	bbox = BBox(p1);
	bbox.expand(p2);
	bbox.expand(p3);

	bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
	// Part 1, Task 3: implement ray-triangle intersection
	// The difference between this function and the next function is that the next
	// function records the "intersection" while this function only tests whether
	// there is a intersection.

	// ray is defined by r(t) = o + td
	Vector3D d = r.d;
	Vector3D o = r.o;
	Vector3D p_prime = this->p1;
	Vector3D N = triangleNormal(p1, p2, p3);

	// using t = ((P'-o) \cdot N) / (d \cdot N)
	// double check parallel ray: dot(d,N) = 0
	double denom = dot(d, N);
	if (abs(denom) < EPS_F) return false; // check for floating points
	// calculate t
	double t = dot((p_prime - o), N) / denom;
	// check ray bounds
	if (t < r.min_t || t > r.max_t) {
		return false;
	}

	// get the intersection point
	Vector3D p_intersect = o + t * d;

	// calculate result of barycentric coordinate
	Vector3D bary = computeBarycentric(p_intersect, p1, p2, p3);

	// check the coefficients
	if (bary[0] < 0 || bary[1] < 0 || bary[2] < 0) return false;

	// Update max_t to the nearest intersection found
	r.max_t = t;

	return true;
}

Vector3D Triangle::triangleNormal(Vector3D p1, Vector3D p2, Vector3D p3) const {
	Vector3D edge1 = p2 - p1;
	Vector3D edge2 = p3 - p1;
	Vector3D N = cross(edge1, edge2);
	N.normalize();
	return N;
}

Vector3D Triangle::computeBarycentric(Vector3D P, Vector3D A, Vector3D B, Vector3D C) const {
	Vector3D v0 = B - A;  // Edge from A to B
	Vector3D v1 = C - A;  // Edge from A to C
	Vector3D v2 = P - A;  // Vector from A to P

	// Compute dot products
	double d00 = dot(v0, v0);
	double d01 = dot(v0, v1);
	double d11 = dot(v1, v1);
	double d20 = dot(v2, v0);
	double d21 = dot(v2, v1);

	// Compute barycentric coordinates
	double denom = d00 * d11 - d01 * d01;
	if (abs(denom) < 1e-12)  return Vector3D(-1.0, -1.0, -1.0);
	double beta = (d11 * d20 - d01 * d21) / denom;
	double gamma = (d00 * d21 - d01 * d20) / denom;
	double alpha = 1.0 - beta - gamma;

	return Vector3D(alpha, beta, gamma);
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
	// Part 1, Task 3:
	// implement ray-triangle intersection. When an intersection takes
	// place, the Intersection data should be updated accordingly
	// ray is defined by r(t) = o + td
	Vector3D d = r.d;
	Vector3D o = r.o;
	Vector3D p_prime = this->p1;
	Vector3D N = triangleNormal(p1, p2, p3);

	double denom = dot(d, N);
	if (abs(denom) < EPS_F) {
		return false;
	}

	double t = dot((p_prime - o), N) / denom;
	//std::cout << "Triangle intersect: t=" << t << ", r.max_t=" << r.max_t << "r.min_t= " << r.min_t << std::endl;
	if (t < r.min_t || t > r.max_t) {
		return false;
	}

	Vector3D p_intersect = o + t * d;

	// calculate result of barycentric coordinate
	Vector3D bary = computeBarycentric(p_intersect, p1, p2, p3);

	// check the coefficients
	if (bary[0] < 0 || bary[1] < 0 || bary[2] < 0) return false;
	//std::cout << "valid triangle hit - barycentric: " << bary << std::endl;
	// Update max_t to the nearest intersection found
	r.max_t = t;

	// update *isect
	isect->t = t;
	// update surface normal
	Vector3D normal_intersect = bary[0] * n1 + bary[1] * n2 + bary[2] * n3;
	isect->n = normal_intersect.unit();
	isect->primitive = this;
	isect->bsdf = this->get_bsdf();
	return true;
}

void Triangle::draw(const Color &c, float alpha) const {
	glColor4f(c.r, c.g, c.b, alpha);
	glBegin(GL_TRIANGLES);
	glVertex3d(p1.x, p1.y, p1.z);
	glVertex3d(p2.x, p2.y, p2.z);
	glVertex3d(p3.x, p3.y, p3.z);
	glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
	glColor4f(c.r, c.g, c.b, alpha);
	glBegin(GL_LINE_LOOP);
	glVertex3d(p1.x, p1.y, p1.z);
	glVertex3d(p2.x, p2.y, p2.z);
	glVertex3d(p3.x, p3.y, p3.z);
	glEnd();
}

} // namespace SceneObjects
} // namespace CGL
