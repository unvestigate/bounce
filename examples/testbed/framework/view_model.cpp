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

#include "view_model.h"
#include "model.h"

#include "GLFW/glfw3.h"

#include "test.h"
#include "tests/convex_hull.h"
#include "tests/cluster.h"
#include "tests/distance_test.h"
#include "tests/linear_time_of_impact.h"
#include "tests/time_of_impact.h"
#include "tests/aabb_time_of_impact.h"
#include "tests/capsule_collision.h"
#include "tests/hull_collision.h"
#include "tests/deep_capsule.h"
#include "tests/box_face_contact.h"
#include "tests/box_edge_contact.h"
#include "tests/linear_motion.h"
#include "tests/angular_motion.h"
#include "tests/gyro_motion.h"
#include "tests/initial_overlap.h"
#include "tests/capsule_spin.h"
#include "tests/quadric_shapes.h"
#include "tests/compound_body.h"
#include "tests/spring_test.h"
#include "tests/motor_test.h"
#include "tests/weld_test.h"
#include "tests/cone_test.h"
#include "tests/revolute_test.h"
#include "tests/prismatic_test.h"
#include "tests/wheel_test.h"
#include "tests/hinge_chain.h"
#include "tests/newton_cradle.h"
#include "tests/ragdoll.h"
#include "tests/mesh_contact_test.h"
#include "tests/triangle_contact_test.h"
#include "tests/hull_contact_test.h"
#include "tests/sphere_stack.h"
#include "tests/capsule_stack.h"
#include "tests/box_stack.h"
#include "tests/sheet_stack.h"
#include "tests/shape_stack.h"
#include "tests/jenga.h"
#include "tests/pyramid.h"
#include "tests/pyramids.h"
#include "tests/ray_cast.h"
#include "tests/shape_cast.h"
#include "tests/sensor_test.h"
#include "tests/body_types.h"
#include "tests/varying_friction.h"
#include "tests/varying_restitution.h"
#include "tests/tumbler.h"
#include "tests/multiple_pendulum.h"
#include "tests/conveyor_belt.h"
#include "tests/shift_center.h"
#include "tests/rope_test.h"

TestSettings* g_testSettings = nullptr;
Settings* g_settings = nullptr;

Settings::Settings()
{
	testID = 0;
	testCount = 0;
	drawPoints = true;
	drawLines = true;
	drawTriangles = true;
	drawGrid = true;
	drawStats = false;
	drawProfiler = false;	
}

void Settings::RegisterTest(const char* name, TestCreate createFn)
{
	TestEntry* test = tests + testCount++;
	test->name = name;
	test->create = createFn;
}
	
ViewModel::ViewModel(Model* model, GLFWwindow* window)
{
	m_model = model;
	m_window = window;
	m_ps0.SetZero();

	m_settings.RegisterTest("Convex Hull", &ConvexHull::Create );
	m_settings.RegisterTest("Cluster", &Cluster::Create );
	m_settings.RegisterTest("Distance", &Distance::Create );
	m_settings.RegisterTest("Linear Time of Impact", &LinearTimeOfImpact::Create );
	m_settings.RegisterTest("Time of Impact", &TimeOfImpact::Create );
	m_settings.RegisterTest("AABB Time of Impact", &AABBTimeOfImpact::Create );
	m_settings.RegisterTest("Capsule Collision", &CapsuleCollision::Create );
	m_settings.RegisterTest("Hull Collision", &HullCollision::Create );
	m_settings.RegisterTest("Deep Capsule", &DeepCapsule::Create );
	m_settings.RegisterTest("Box Face Contact", &BoxFaceContact::Create );
	m_settings.RegisterTest("Box Edge Contact", &BoxEdgeContact::Create );
	m_settings.RegisterTest("Capsule Spin", &CapsuleSpin::Create );
	m_settings.RegisterTest("Hull Contact Test", &HullContactTest::Create );
	m_settings.RegisterTest("Triangle Contact Test", &TriangleContactTest::Create );
	m_settings.RegisterTest("Mesh Contact Test", &MeshContactTest::Create );
	m_settings.RegisterTest("Linear Motion", &LinearMotion::Create );
	m_settings.RegisterTest("Angular Motion", &AngularMotion::Create );
	m_settings.RegisterTest("Gyroscopic Motion", &GyroMotion::Create );
	m_settings.RegisterTest("Compound Body", &CompoundBody::Create );
	m_settings.RegisterTest("Quadric Shapes", &QuadricShapes::Create );
	m_settings.RegisterTest("Spring Test", &SpringTest::Create );
	m_settings.RegisterTest("Prismatic Test", &PrismaticTest::Create );
	m_settings.RegisterTest("Wheel Test", &WheelTest::Create );
	m_settings.RegisterTest("Weld Test", &WeldTest::Create );
	m_settings.RegisterTest("Cone Test", &ConeTest::Create );
	m_settings.RegisterTest("Motor Test", &MotorTest::Create );
	m_settings.RegisterTest("Revolute Test", &RevoluteTest::Create );
	m_settings.RegisterTest("Hinge Chain", &HingeChain::Create );
	m_settings.RegisterTest("Ragdoll", &Ragdoll::Create );
	m_settings.RegisterTest("Newton's Cradle", &NewtonCradle::Create );
	m_settings.RegisterTest("Sphere Stack", &SphereStack::Create );
	m_settings.RegisterTest("Capsule Stack", &CapsuleStack::Create );
	m_settings.RegisterTest("Box Stack", &BoxStack::Create );
	m_settings.RegisterTest("Sheet Stack", &SheetStack::Create );
	m_settings.RegisterTest("Shape Stack", &ShapeStack::Create );
	m_settings.RegisterTest("Jenga", &Jenga::Create );
	m_settings.RegisterTest("Box Pyramid", &Pyramid::Create );
	m_settings.RegisterTest("Box Pyramid Rows", &Pyramids::Create );
	m_settings.RegisterTest("Ray Cast", &RayCast::Create );
	m_settings.RegisterTest("Shape Cast", &ShapeCast::Create );
	m_settings.RegisterTest("Sensor Test", &SensorTest::Create );
	m_settings.RegisterTest("Body Types", &BodyTypes::Create );
	m_settings.RegisterTest("Varying Friction", &VaryingFriction::Create );
	m_settings.RegisterTest("Varying Restitution", &VaryingRestitution::Create );
	m_settings.RegisterTest("Tumbler", &Tumbler::Create );
	m_settings.RegisterTest("Initial Overlap", &InitialOverlap::Create );
	m_settings.RegisterTest("Multiple Pendulum", &MultiplePendulum::Create );
	m_settings.RegisterTest("Conveyor Belt", &ConveyorBelt::Create );
	m_settings.RegisterTest("Shift Center", &ShiftCenter::Create);
	m_settings.RegisterTest("Rope", &Rope::Create);

	g_settings = &m_settings;
	g_testSettings = &m_testSettings;
}

ViewModel::~ViewModel()
{
	g_settings = nullptr;
	g_testSettings = nullptr;
}

b3Vec2 ViewModel::GetCursorPosition() const
{
	double x, y;
	glfwGetCursorPos(m_window, &x, &y);
	return b3Vec2(scalar(x), scalar(y));
}

void ViewModel::Action_SetTest()
{
	m_model->Action_SetTest();
}

void ViewModel::Action_PreviousTest()
{
	m_settings.testID = b3Clamp(m_settings.testID - 1, 0, int(m_settings.testCount) - 1);
	m_model->Action_SetTest();
}

void ViewModel::Action_NextTest()
{
	m_settings.testID = b3Clamp(m_settings.testID + 1, 0, int(m_settings.testCount) - 1);
	m_model->Action_SetTest();
}

void ViewModel::Action_PlayPause()
{
	m_testSettings.pause = !m_testSettings.pause;
}

void ViewModel::Action_SinglePlay()
{
	m_testSettings.pause = true;
	m_testSettings.singlePlay = true;
}

void ViewModel::Action_ResetCamera()
{
	m_model->Action_ResetCamera();
}

void ViewModel::Event_SetWindowSize(int w, int h)
{
	m_model->Command_ResizeCamera(scalar(w), scalar(h));
}

void ViewModel::Event_Press_Key(int button)
{
	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (shiftDown)
	{
		if (button == GLFW_KEY_DOWN)
		{
			m_model->Command_ZoomCamera(1.0f);
		}

		if (button == GLFW_KEY_UP)
		{
			m_model->Command_ZoomCamera(-1.0f);
		}
	}
	else
	{
		m_model->Command_Press_Key(button);
	}
}

void ViewModel::Event_Release_Key(int button)
{
	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (!shiftDown)
	{
		m_model->Command_Release_Key(button);
	}
}

void ViewModel::Event_Press_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
		if (!shiftDown)
		{
			m_model->Command_Press_Mouse_Left(GetCursorPosition());
		}
	}
}

void ViewModel::Event_Release_Mouse(int button)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
		if (!shiftDown)
		{
			m_model->Command_Release_Mouse_Left(GetCursorPosition());
		}
	}
}

void ViewModel::Event_Move_Cursor(float x, float y)
{
	b3Vec2 ps;
	ps.Set(x, y);

	b3Vec2 dp = ps - m_ps0;

	m_ps0 = ps;

	b3Vec2 n = b3Normalize(dp);

	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	bool leftDown = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
	bool rightDown = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

	if (shiftDown)
	{
		if (leftDown)
		{
			scalar ax = -0.005f * B3_PI * n.x;
			scalar ay = -0.005f * B3_PI * n.y;

			m_model->Command_RotateCameraY(ax);
			m_model->Command_RotateCameraX(ay);
		}

		if (rightDown)
		{
			scalar tx = 0.2f * n.x;
			scalar ty = -0.2f * n.y;

			m_model->Command_TranslateCameraX(tx);
			m_model->Command_TranslateCameraY(ty);
		}
	}
	else
	{
		m_model->Command_Move_Cursor(ps);
	}
}

void ViewModel::Event_Scroll(float dx, float dy)
{
	b3Vec2 n(dx, dy);
	n.Normalize();

	bool shiftDown = glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
	if (shiftDown)
	{
		m_model->Command_ZoomCamera(1.0f * n.y);
	}
}
