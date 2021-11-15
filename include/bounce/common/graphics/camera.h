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

#ifndef B3_CAMERA_H
#define B3_CAMERA_H

#include <bounce/common/math/vec2.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/math/mat44.h>

// An spherical camera controller (also called orbit camera).
// This accepts spherical and Cartesian coordinates as input.
// See https://en.wikipedia.org/wiki/Spherical_coordinate_system
class b3Camera
{
public:
	// Default ctor.
	b3Camera();
	
	// Set the camera width.
	void SetWidth(scalar width);
	
	// Get the camera width.
	scalar GetWidth() const { return m_width; } 
	
	// Set the camera height.
	void SetHeight(scalar height);
	
	// Get the camera height.
	scalar GetHeight() const { return m_height; }
	
	// Set the near and far plane distances.
	void SetDistances(scalar zNear, scalar zFar);
	
	// Get the near plane distance.
	scalar GetZNear() const { return m_zNear; }

	// Get the far plane distance.
	scalar GetZFar() const { return m_zFar; }
	
	// Set the full field of view angle in the range [0, pi].
	void SetYFOV(scalar yFov);
	
	// Get the full field of view angle.
	scalar GetYFOV() const { return m_yFov; }
	
	// Set the always-positive radius coordinate. 
	void SetRadius(scalar radius);
	
	// Get the radius coordinate.
	scalar GetRadius() const { return m_r; }
	
	// Set the polar angle in the range [0, pi].
	void SetPolarAngle(scalar angle);
	
	// Get the polar angle in the range [0, pi].
	scalar GetPolarAngle() const { return m_theta; }
	
	// Set the azimuthal angle in the range [0, 2*pi].
	void SetAzimuthalAngle(scalar angle);
	
	// Get the azimuthal angle in the range [0, 2*pi].
	scalar GetAzimuthalAngle() const { return m_phi; }
	
	// Set the sphere center.
	void SetCenter(const b3Vec3& center) { m_center = center; }
	
	// Get the sphere center.
	const b3Vec3& GetCenter() const { return m_center; }
	
	// Translate the sphere center in the direction of the camera x axis.
	void TranslateXAxis(scalar distance);
	
	// Translate the sphere center in the direction of the camera y axis.
	void TranslateYAxis(scalar distance);
	
	// Translate the sphere center in the direction of the camera z axis.
	void TranslateZAxis(scalar distance);
	
	// Increase/decrease the radius.
	void AddRadius(scalar distance);
	
	// Increase/decrease the polar angle.
	void AddPolarAngle(scalar angle);
	
	// Increase/decrease the azimuthal angle.
	void AddAzimuthalAngle(scalar angle);	
	
	// Look at a given target position from a given eye position.
	void LookAt(const b3Vec3& eyePosition, const b3Vec3& targetPosition);
	
	// Set the camera position from three Cartesian coordinates.
	void SetPosition(const b3Vec3& pw);
	
	// Get the camera position in Cartesian coordinates.
	b3Vec3 BuildPosition() const;
	
	// Get the camera x axis.
	b3Vec3 BuildXAxis() const;
	
	// Get the camera y axis.
	b3Vec3 BuildYAxis() const;
	
	// Get the camera z axis.
	b3Vec3 BuildZAxis() const;
	
	// Get the camera rotation matrix.
	b3Mat33 BuildRotation() const;
	
	// Get the camera view matrix.
	b3Mat44 BuildViewMatrix() const;
	
	// Get the camera projection matrix.
	b3Mat44 BuildProjectionMatrix() const;
	
	// Convert a point in world space to screen space.
	b3Vec2 ConvertWorldToScreen(const b3Vec3& pw) const;
	
	// Convert a point in screen space to world space.
	b3Vec3 ConvertScreenToWorld(const b3Vec2& ps) const;
private:
	// Projection parameters
	scalar m_width, m_height;
	scalar m_zNear, m_zFar;
	scalar m_yFov;
	
	// Radius
	scalar m_r;

	// Polar angle
	scalar m_theta;

	// Azimuthal angle
	scalar m_phi;

	// Center 
	b3Vec3 m_center;
};

#endif