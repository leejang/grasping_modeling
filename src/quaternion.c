#include <math.h>

#ifndef QUATERNION_C
#define QUATERNION_C

typedef struct {
	double w;
	double x;
	double y;
	double z;
} quaternion_t;

const quaternion_t QUATERNION_UNITY = {1, 0, 0, 0};

typedef struct {
	double x;
	double y;
	double z;
} xyz_t;

const xyz_t AXIS_X = {1, 0, 0};
const xyz_t AXIS_Y = {0, 1, 0};
const xyz_t AXIS_Z = {0, 0, 1};

// BASIC QUATERNION MATH
// Magnitude {{{

double quaternion_mag (quaternion_t q) {
	return sqrt(pow(q.w, 2) + pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2));
}

// }}}
// Multiplication {{{

quaternion_t quaternion_mul (quaternion_t q, quaternion_t r) {
	quaternion_t qr; // product q times r

	qr.w = r.w * q.w - r.x * q.x - r.y * q.y - r.z * q.z;
	qr.x = r.w * q.x + r.x * q.w - r.y * q.z + r.z * q.y;
	qr.y = r.w * q.y + r.x * q.z + r.y * q.w - r.z * q.x;
	qr.z = r.w * q.z - r.x * q.y + r.y * q.x + r.z * q.w;

	return qr;
}

// }}}
// Inverse {{{

quaternion_t quaternion_inv (quaternion_t q) {
	quaternion_t p; // inverse

	double magsq = pow(q.w, 2) + pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2);

	p.w =   q.w / magsq;
	p.x = - q.x / magsq;
	p.y = - q.y / magsq;
	p.z = - q.z / magsq;

	return p;
}

// }}}
// Division {{{

// Does this function compute the attitude difference from unit quaternion a
// to unit quaternion b ???
// NO!  It does not.  It merely computes the Hamiltonian quotient of a and b.
quaternion_t quaternion_div (quaternion_t b, quaternion_t a) {
	quaternion_t dq;

	float asquared = pow(a.w, 2) + pow(a.x, 2) + pow(a.y, 2) + pow(a.z, 2);

	dq.w = (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z) / asquared;
	dq.x = (a.w * b.x - a.x * b.w - a.y * b.z + a.z * b.y) / asquared;
	dq.y = (a.w * b.y + a.x * b.z - a.y * b.w - a.z * b.x) / asquared;
	dq.z = (a.w * b.z + a.x * b.y - a.y * b.x - a.z * b.w) / asquared;

	return dq;
}

// }}}
// Relate {{{

// rephrases the attitude quaternion ``absol'' with respect to the attitude
// quaternion ``tare''
quaternion_t quaternion_relate (quaternion_t absol, quaternion_t tare) {
	return quaternion_mul(absol, quaternion_inv(tare));
}

// }}}

// QUATERNION ROTATIONS
// Construct a quaternion from an axis and an angle {{{

// Returns a quaternion to represent a rotation of angle theta
// (in radians) around the given axis
quaternion_t quaternion_from_axis_angle (xyz_t axis, double theta) {
	quaternion_t q;

	q.w = cos(theta / 2);
	double coeff = sin(theta / 2);
	q.x = coeff * axis.x;
	q.y = coeff * axis.y;
	q.z = coeff * axis.z;

	return q;
}

// }}}
// Apply a quaternion rotation to a vector {{{

// Returns the vector v that is the image of u under the rotation represented by q
xyz_t quaternion_rotate (quaternion_t q, xyz_t u) {
	quaternion_t u_quat = {0, u.x, u.y, u.z};
	quaternion_t v_quat = quaternion_mul(q, quaternion_mul(u_quat, quaternion_inv(q)));

	xyz_t v = {v_quat.x, v_quat.y, v_quat.z};
	return v;
}

// }}}

// VECTOR OPERATIONS
// Vector addition {{{

xyz_t xyz_add (xyz_t a, xyz_t b) {
	xyz_t sum = {a.x + b.x, a.y + b.y, a.z + b.z};
	return sum;
}

// }}}
// Scalar multiplication {{{

xyz_t xyz_scale (xyz_t vec, double alpha) {
	xyz_t scaled = {vec.x * alpha, vec.y * alpha, vec.z * alpha};
	return scaled;
}

#endif
