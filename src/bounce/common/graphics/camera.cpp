/*
* Copyright (c) 2016-2019 Irlan Robson
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <bounce/common/graphics/camera.h>

b3Camera::b3Camera()
{
	m_width = scalar(1280);
	m_height = scalar(720);
	m_zNear = scalar(1);
	m_zFar = scalar(1000);
	m_yFov = scalar(0.25) * B3_PI;
	m_r = scalar(1);
	m_theta = scalar(0);
	m_phi = scalar(0);
	m_center.SetZero();
}

void b3Camera::SetWidth(scalar width) 
{ 
	B3_ASSERT(width > scalar(0));
	m_width = width; 
}

void b3Camera::SetHeight(scalar height) 
{ 
	B3_ASSERT(height > scalar(0));
	m_height = height;
}

void b3Camera::SetDistances(scalar zNear, scalar zFar)
{
	B3_ASSERT(zNear < zFar);
	B3_ASSERT(zNear >= scalar(0));
	B3_ASSERT(zFar > scalar(0));
	m_zNear = zNear;
	m_zFar = zFar;
}

void b3Camera::SetYFOV(scalar yFov) 
{ 
	B3_ASSERT(yFov > scalar(0));
	B3_ASSERT(yFov < B3_PI);
	m_yFov = yFov;
};

void b3Camera::SetPolarAngle(scalar angle)
{
	B3_ASSERT(angle >= scalar(0));
	B3_ASSERT(angle <= B3_PI);
	m_theta = angle;
}

void b3Camera::SetAzimuthalAngle(scalar angle)
{
	B3_ASSERT(angle >= scalar(0));
	B3_ASSERT(angle <= scalar(2) * B3_PI);
	m_phi = angle;
}

void b3Camera::TranslateXAxis(scalar distance)
{
	b3Vec3 x = BuildXAxis();
	m_center += distance * x;
}

void b3Camera::TranslateYAxis(scalar distance)
{
	b3Vec3 y = BuildYAxis();
	m_center += distance * y;
}

void b3Camera::TranslateZAxis(scalar distance)
{
	b3Vec3 z = BuildZAxis();
	m_center += distance * z;
}

void b3Camera::SetRadius(scalar radius) 
{ 
	B3_ASSERT(radius >= scalar(0));
	m_r = radius;
}

void b3Camera::AddRadius(scalar distance)
{
	if (m_r + distance < scalar(0))
	{
		m_r = scalar(0);
	}
	else
	{
		m_r += distance;
	}
}

void b3Camera::LookAt(const b3Vec3& eyePosition, const b3Vec3& targetPosition)
{
	SetPosition(eyePosition);
	m_center = targetPosition;
}

void b3Camera::AddPolarAngle(scalar angle)
{
	if (m_theta + angle < scalar(0))
	{
		m_theta = scalar(0);
	}
	else if (m_theta + angle > B3_PI)
	{
		m_theta = B3_PI;
	}
	else
	{
		m_theta += angle;
	}
}

void b3Camera::AddAzimuthalAngle(scalar angle)
{
	m_phi += angle;

	// Normalize angle in range [0, 2*pi]
	scalar two_pi = scalar(2) * B3_PI;
	m_phi -= floor(m_phi / two_pi) * two_pi;
}

// This detects the quadrant the given point is on  
// and returns the corresponding angle in the range [0, 2*pi].
static scalar b3AzimuthalAngle(scalar x, scalar y)
{
	if (x > scalar(0) && y > scalar(0))
	{
		return atan(y / x);
	}

	if (x < scalar(0) && y > scalar(0))
	{
		return scalar(0.5) * B3_PI + atan(-x / y);
	}

	if (x < scalar(0) && y < scalar(0))
	{
		return B3_PI + atan(y / x);
	}

	if (x > scalar(0) && y < scalar(0))
	{
		return B3_PI + scalar(0.5) * B3_PI + atan(-x / y);
	}

	if (x == scalar(0) && y > scalar(0))
	{
		return scalar(0.5) * B3_PI;
	}

	if (x == scalar(0) && y < scalar(0))
	{
		return B3_PI + scalar(0.5) * B3_PI;
	}

	if (x > scalar(0) && y == scalar(0))
	{
		return scalar(0);
	}

	if (x < scalar(0) && y == scalar(0))
	{
		return B3_PI;
	}

	if (x == scalar(0) && y == scalar(0))
	{
		return scalar(0);
	}

	B3_ASSERT(false);
	return scalar(0);
}

// This detects the quadrant the given point is on  
// and returns the corresponding angle in the range [0, pi].
static scalar b3PolarAngle(scalar x, scalar y)
{
	if (x > scalar(0) && y > scalar(0))
	{
		return atan(y / x);
	}

	if (x < scalar(0) && y > scalar(0))
	{
		return scalar(0.5) * B3_PI + atan(-x / y);
	}

	if (x < scalar(0) && y < scalar(0))
	{
		return atan(x / y);
	}

	if (x > scalar(0) && y < scalar(0))
	{
		return atan(-y / x);
	}

	if (x == scalar(0) && y > scalar(0))
	{
		return scalar(0.5) * B3_PI;
	}

	if (x == scalar(0) && y < scalar(0))
	{
		return scalar(0.5) * B3_PI;
	}

	if (y == scalar(0) && x > scalar(0))
	{
		return scalar(0);
	}

	if (y == scalar(0) && x < scalar(0))
	{
		return B3_PI;
	}

	if (x == scalar(0) && y == scalar(0))
	{
		return scalar(0);
	}

	B3_ASSERT(false);
	return scalar(0);
}

void b3Camera::SetPosition(const b3Vec3& pw)
{
	scalar x = pw.x, y = pw.y, z = pw.z;

	scalar r = b3Sqrt(x * x + y * y + z * z);

	scalar phi = b3AzimuthalAngle(z, x);
	B3_ASSERT(phi >= scalar(0) && phi <= scalar(2) * B3_PI);

	scalar s = b3Sqrt(z * z + x * x);
	scalar theta = b3PolarAngle(y, s);
	B3_ASSERT(theta >= scalar(0) && theta <= B3_PI);

	m_r = r;
	m_phi = phi;
	m_theta = theta;
}

b3Vec3 b3Camera::BuildPosition() const
{
	b3Vec3 z = BuildZAxis();
	return m_center + m_r * z;
}

b3Vec3 b3Camera::BuildXAxis() const
{
	b3Vec3 x;
	x.x = cos(m_phi);
	x.y = scalar(0);
	x.z = -sin(m_phi);
	return x;
}

b3Vec3 b3Camera::BuildYAxis() const
{
	b3Vec3 y;
	y.x = -cos(m_theta) * sin(m_phi);
	y.y = sin(m_theta);
	y.z = -cos(m_theta) * cos(m_phi);
	return y;
}

b3Vec3 b3Camera::BuildZAxis() const
{
	b3Vec3 z;
	z.x = sin(m_theta) * sin(m_phi);
	z.y = cos(m_theta);
	z.z = sin(m_theta) * cos(m_phi);
	return z;
}

b3Mat33 b3Camera::BuildRotation() const
{
	b3Vec3 x = BuildXAxis();
	b3Vec3 y = BuildYAxis();
	b3Vec3 z = BuildZAxis();
	
	return b3Mat33(x, y, z);
}

b3Mat44 b3Camera::BuildViewMatrix() const
{
	b3Vec3 translation = BuildPosition();
	b3Mat33 rotation = BuildRotation();

	b3Mat33 R = b3Transpose(rotation);
	b3Vec3 t = -(R * translation);

	b3Mat44 V;
	V.x.Set(R.x.x, R.x.y, R.x.z, scalar(0));
	V.y.Set(R.y.x, R.y.y, R.y.z, scalar(0));
	V.z.Set(R.z.x, R.z.y, R.z.z, scalar(0));
	V.w.Set(t.x, t.y, t.z, scalar(1));
	return V;
}

b3Mat44 b3Camera::BuildProjectionMatrix() const
{
	scalar w = m_width, h = m_height;
	scalar zn = m_zNear, zf = m_zFar;
	scalar yfov = m_yFov;

	scalar ratio = w / h;

	scalar t = tan(scalar(0.5) * yfov);
	scalar sx = scalar(1) / (ratio * t);
	scalar sy = scalar(1) / t;

	scalar sz = (zn + zf) / (zn - zf);
	scalar tz = (zf * zn) / (zn - zf);

	b3Mat44 P;
	P.x.Set(sx, scalar(0), scalar(0), scalar(0));
	P.y.Set(scalar(0), sy, scalar(0), scalar(0));
	P.z.Set(scalar(0), scalar(0), sz, scalar(-1));
	P.w.Set(scalar(0), scalar(0), tz, scalar(0));
	return P;
}

b3Vec2 b3Camera::ConvertWorldToScreen(const b3Vec3& pw3) const
{
	scalar w = m_width, h = m_height;

	b3Mat44 P = BuildProjectionMatrix();
	b3Mat44 V = BuildViewMatrix();

	b3Vec4 pw(pw3.x, pw3.y, pw3.z, scalar(1));

	b3Vec4 pp = P * V * pw;

	b3Vec3 pn(pp.x, pp.y, pp.z);
	scalar inv_w = pp.w != scalar(0) ? scalar(1) / pp.w : scalar(1);
	pn = inv_w * pn;

	scalar u = scalar(0.5) * (pn.x + scalar(1));
	scalar v = scalar(0.5) * (pn.y + scalar(1));

	b3Vec2 ps;
	ps.x = u * w;
	ps.y = (scalar(1) - v) * h;
	return ps;
}

b3Vec3 b3Camera::ConvertScreenToWorld(const b3Vec2& ps) const
{
	scalar w = m_width, h = m_height;

	scalar t = tan(scalar(0.5) * m_yFov);
	scalar ratio = w / h;

	b3Vec3 vv;
	vv.x = scalar(2) * ratio * ps.x / w - ratio;
	vv.y = -scalar(2) * ps.y / h + scalar(1);
	vv.z = -scalar(1) / t;

	b3Mat33 R = BuildRotation();

	b3Vec3 vw = R * vv;

	return b3Normalize(vw);
}
