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

#include "view.h"
#include "view_model.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "GLFW/glfw3.h"

#include <bounce/common/profiler.h>
#include <bounce/common/graphics/camera.h>
#include <bounce/common/graphics/color.h>

#include <stdio.h>

static inline bool GetTestName(void* userData, int idx, const char** name)
{
	assert(idx < g_settings->testCount);
	*name = g_settings->tests[idx].name;
	return true;
}

void DrawString(const b3Color& color, const b3Vec3& pw, const char* string, ...)
{
	extern b3Camera* g_camera;

	b3Vec2 ps = g_camera->ConvertWorldToScreen(pw);
	
	va_list args;
	va_start(args, string);
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(g_camera->GetWidth(), g_camera->GetHeight()));
	ImGui::Begin("Superlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(ps.x, ps.y));
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), string, args);
	ImGui::End();
	va_end(args);
}

void DrawString(const b3Color& color, const char* string, ...)
{
	va_list args;
	va_start(args, string);

	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::SetNextWindowPos(ImVec2(0.0f, 40.0f));
	ImGui::SetNextWindowSize(ImVec2(0.0f, 0.0f));
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::TextColoredV(ImVec4(color.r, color.g, color.b, color.a), string, args);
	ImGui::End();
	ImGui::PopStyleVar();

	va_end(args);	
}

View::View(ViewModel* viewModel, GLFWwindow* window, const char* glslVersion)
{
	m_viewModel = viewModel;
	m_window = window;

	// Create UI
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	bool success = ImGui_ImplGlfw_InitForOpenGL(m_window, false);
	if (success == false)
	{
		printf("ImGui_ImplGlfw_InitForOpenGL failed.\n");
		assert(false);
	}

	success = ImGui_ImplOpenGL3_Init(glslVersion);
	if (success == false)
	{
		printf("ImGui_ImplOpenGL3_Init failed.\n");
		assert(false);
	}

}

View::~View()
{
	// Destroy UI
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

void View::Event_SetWindowSize(int w, int h)
{
	m_viewModel->Event_SetWindowSize(w, h);
}

void View::Event_Press_Key(int button)
{
	m_viewModel->Event_Press_Key(button);
}

void View::Event_Release_Key(int button)
{
	m_viewModel->Event_Release_Key(button);
}

void View::Event_Press_Mouse(int button)
{
	m_viewModel->Event_Press_Mouse(button);
}

void View::Event_Release_Mouse(int button)
{
	m_viewModel->Event_Release_Mouse(button);
}

void View::Event_Move_Cursor(float x, float y)
{
	m_viewModel->Event_Move_Cursor(x, y);
}

void View::Event_Scroll(float dx, float dy)
{
	m_viewModel->Event_Scroll(dx, dy);
}

void View::BeginInterface()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

static void TreeNode(const b3ProfilerNode* node, uint32& index)
{
	ImGui::PushID(index);
	++index;

	if (ImGui::TreeNode(node->GetName()))
	{
		double elapsed = node->GetElapsedTime();
		uint32 callCount = node->GetCallCount();
		const b3ProfilerStatistic* stats = node->GetStatistic();

		ImGui::Text("(elapsed = %.4f) (min = %.4f) (max = %.4f) (calls = %d) [ms]", elapsed, stats->GetMinElapsed(), stats->GetMaxElapsed(), callCount);

		for (const b3ProfilerNode* n = node->GetChildList(); n != nullptr; n = n->GetNextChild())
		{
			TreeNode(n, index);
		}
		ImGui::TreePop();
	}

	ImGui::PopID();
}

void View::Interface()
{
	Settings& settings = m_viewModel->m_settings;

	bool openControls = false;
	bool openAbout = false;
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("Exit", "Alt+F4"))
			{
				glfwSetWindowShouldClose(m_window, true);
			}

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("View"))
		{
			ImGui::MenuItem("Statistics", "", &settings.drawStats);
			ImGui::MenuItem("Profiler", "", &settings.drawProfiler);

			ImGui::Separator();

			ImGui::MenuItem("Points", "", &settings.drawPoints);
			ImGui::MenuItem("Lines", "", &settings.drawLines);
			ImGui::MenuItem("Triangles", "", &settings.drawTriangles);

			ImGui::Separator();
			
			ImGui::MenuItem("Reference Grid", "", &settings.drawGrid);

			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Help"))
		{
			if (ImGui::MenuItem("Controls"))
			{
				openControls = true;
			}
			
			if (ImGui::MenuItem("About"))
			{
				openAbout = true;
			}

			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}
	
	if (openControls)
	{
		ImGui::OpenPopup("Controls");
	}

	if (openAbout)
	{
		ImGui::OpenPopup("About Bounce Testbed");
	}

	ImVec2 buttonSize(-1.0f, 0.0f);
	
	if (ImGui::BeginPopupModal("Controls", NULL, ImGuiWindowFlags_Popup | ImGuiWindowFlags_NoResize))
	{
		ImGui::Text("Rotate the scene using LSHIFT + LMB");
		ImGui::Text("Translate the scene using LSHIFT + RMB");
		ImGui::Text("Zoom in / out the scene using LSHIFT + Mouse Wheel");

		if (ImGui::Button("OK", buttonSize))
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	if (ImGui::BeginPopupModal("About Bounce Testbed", NULL, ImGuiWindowFlags_Popup | ImGuiWindowFlags_NoResize))
	{
		extern b3Version b3_version;

		ImGui::Text("Bounce Testbed");
		ImGui::Text("Version %d.%d.%d", b3_version.major, b3_version.minor, b3_version.revision);
		ImGui::Text("Copyright (c) Irlan Robson");
		
		if (ImGui::Button("OK", buttonSize))
		{
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}

	int width, height;
	glfwGetWindowSize(m_window, &width, &height);
	
	ImGui::SetNextWindowPos(ImVec2(0.0f, 20.0f));
	ImGui::SetNextWindowSize(ImVec2(float(width), 20.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(0.0f, 0.0f));

	ImGui::Begin("##ToolBar", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar);

	if (ImGui::BeginMenuBar())
	{
		ImGui::PushItemWidth(250.0f);

		ImGui::Separator();
		
		if (ImGui::Combo("##Test", &settings.testID, GetTestName, NULL, settings.testCount, settings.testCount))
		{
			m_viewModel->Action_SetTest();
		}

		ImGui::PopItemWidth();

		ImVec2 menuButtonSize(100.0f, 0.0f);

		ImGui::Separator();

		if (ImGui::Button("Previous", menuButtonSize))
		{
			m_viewModel->Action_PreviousTest();
		}

		if (ImGui::Button("Next", menuButtonSize))
		{
			m_viewModel->Action_NextTest();
		}

		ImGui::Separator();

		if (ImGui::Button("Restart", menuButtonSize))
		{
			m_viewModel->Action_SetTest();
		}

		ImGui::Separator();

		if (ImGui::Button("Reset Camera", menuButtonSize))
		{
			m_viewModel->Action_ResetCamera();
		}

		ImGui::EndMenuBar();
	}

	ImGui::End();
	
	ImGui::PopStyleVar();

	TestSettings& testSettings = m_viewModel->m_testSettings;
	
	ImGui::SetNextWindowPos(ImVec2(width - 250.0f, 40.0f));
	ImGui::SetNextWindowSize(ImVec2(250.0f, height - 40.0f));
	ImGui::Begin("Test Settings", NULL, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

	ImGui::PushItemWidth(-1.0f);

	ImGui::Text("Hertz");
	ImGui::SliderFloat("##Hertz", &testSettings.hertz, 0.0f, 240.0f, "%.1f");

	ImGui::Text("Velocity Iterations");
	ImGui::SliderInt("##Velocity Iterations", &testSettings.velocityIterations, 0, 50);

	ImGui::Text("Position Iterations");
	ImGui::SliderInt("##Position Iterations", &testSettings.positionIterations, 0, 50);

	ImGui::Checkbox("Sleep", &testSettings.sleep);
	ImGui::Checkbox("Convex Cache", &testSettings.convexCache);
	ImGui::Checkbox("Warm Start", &testSettings.warmStart);

	if (ImGui::Button("Play/Pause", buttonSize))
	{
		m_viewModel->Action_PlayPause();
	}

	if (ImGui::Button("Single Play", buttonSize))
	{
		m_viewModel->Action_SinglePlay();
	}

	ImGui::Separator();
	
	ImGui::Text("View");

	ImGui::MenuItem("Center of Masses", "", &testSettings.drawCenterOfMasses);
	ImGui::MenuItem("Bounding Boxes", "", &testSettings.drawBounds);
	ImGui::MenuItem("Shapes", "", &testSettings.drawShapes);
	ImGui::MenuItem("Joints", "", &testSettings.drawJoints);
	ImGui::MenuItem("Contact Points", "", &testSettings.drawContactPoints);
	ImGui::MenuItem("Contact Normals", "", &testSettings.drawContactNormals);
	ImGui::MenuItem("Contact Tangents", "", &testSettings.drawContactTangents);
	ImGui::MenuItem("Contact Polygons", "", &testSettings.drawContactPolygons);

	ImGui::PopItemWidth();

	ImGui::End();

	if (g_settings->drawProfiler)
	{
		ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
		ImVec2 wp = ImGui::GetWindowPos();
		ImVec2 ws = ImGui::GetWindowSize();
		ImGui::End();

		wp.y = wp.y + ws.y;

		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

		int width, height;
		glfwGetWindowSize(m_window, &width, &height);

		ImGui::SetNextWindowBgAlpha(0.0f);
		ImGui::SetNextWindowPos(ImVec2(0.0f, wp.y));
		ImGui::SetNextWindowSize(ImVec2(width - 250.0f, 0.0f));

		ImGui::Begin("Profiler", NULL, ImGuiWindowFlags_AlwaysAutoResize);

		extern b3Profiler* g_profiler;
		
		const b3ProfilerNode* root = g_profiler->GetRoot();
		if (root)
		{
			uint32 index = 0;
			TreeNode(root, index);
		}

		ImGui::End();

		ImGui::PopStyleVar();
	}
}

void View::RenderInterface()
{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
