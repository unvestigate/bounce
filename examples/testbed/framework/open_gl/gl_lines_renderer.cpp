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

#include "gl_lines_renderer.h"
#include "gl_shader.h"

#include <stdlib.h>
#include <assert.h>

GLLinesRenderer::GLLinesRenderer(int line_capacity)
{
	const char* vs = \
		"#version 330\n"
		"uniform mat4 projectionMatrix;\n"
		"layout(location = 0) in vec3 v_position;\n"
		"layout(location = 1) in vec4 v_color;\n"
		"out vec4 f_color;\n"
		"void main(void)\n"
		"{\n"
		"	f_color = v_color;\n"
		"	gl_Position = projectionMatrix * vec4(v_position, 1.0f);\n"
		"}\n";

	const char* fs = \
		"#version 330\n"
		"in vec4 f_color;\n"
		"out vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"	color = f_color;\n"
		"}\n";

	m_program = GLCreateShaderProgram(vs, fs);
	m_projection_uniform = glGetUniformLocation(m_program, "projectionMatrix");
	m_position_attribute = 0;
	m_color_attribute = 1;

	m_vertex_capacity = 2 * line_capacity;
	m_positions = (float*)malloc(m_vertex_capacity * 3 * sizeof(float));
	m_colors = (float*)malloc(m_vertex_capacity * 4 * sizeof(float));
	m_vertex_count = 0;

	// Generate
	glGenVertexArrays(1, &m_vao);
	glGenBuffers(2, m_vbos);

	glBindVertexArray(m_vao);
	glEnableVertexAttribArray(m_position_attribute);
	glEnableVertexAttribArray(m_color_attribute);

	// Vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, m_vbos[0]);
	glVertexAttribPointer(m_position_attribute, 3, GL_FLOAT, GL_FALSE, 0, GL_PTR_ADD(NULL, 0));
	glBufferData(GL_ARRAY_BUFFER, m_vertex_capacity * 3 * sizeof(float), m_positions, GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbos[1]);
	glVertexAttribPointer(m_color_attribute, 4, GL_FLOAT, GL_FALSE, 0, GL_PTR_ADD(NULL, 0));
	glBufferData(GL_ARRAY_BUFFER, m_vertex_capacity * 4 * sizeof(float), m_colors, GL_DYNAMIC_DRAW);

	GLCheckGLError();

	// Cleanup
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			if (i == j)
			{
				m_mvp[i + 4 * j] = 1.0f;
			}
			else
			{
				m_mvp[i + 4 * j] = 0.0f;
			}
		}
	}
}

GLLinesRenderer::~GLLinesRenderer()
{
	glDeleteProgram(m_program);
	glDeleteBuffers(2, m_vbos);
	glDeleteVertexArrays(1, &m_vao);

	free(m_positions);
	free(m_colors);
}

void GLLinesRenderer::Vertex(float x, float y, float z, float r, float g, float b, float a)
{
	if (m_vertex_count == m_vertex_capacity)
	{
		Flush();
	}

	m_positions[3 * m_vertex_count] = x;
	m_positions[3 * m_vertex_count + 1] = y;
	m_positions[3 * m_vertex_count + 2] = z;
	m_colors[4 * m_vertex_count] = r;
	m_colors[4 * m_vertex_count + 1] = g;
	m_colors[4 * m_vertex_count + 2] = b;
	m_colors[4 * m_vertex_count + 3] = a;
	++m_vertex_count;
}

void GLLinesRenderer::SetMVP(scalar* mvp)
{
	for (int i = 0; i < 16; ++i)
	{
		m_mvp[i] = (float)mvp[i];
	}
}

void GLLinesRenderer::Flush()
{
	if (m_vertex_count == 0)
	{
		return;
	}

	glUseProgram(m_program);

	glUniformMatrix4fv(m_projection_uniform, 1, GL_FALSE, m_mvp);

	glBindVertexArray(m_vao);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbos[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, m_vertex_count * 3 * sizeof(float), m_positions);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbos[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, m_vertex_count * 4 * sizeof(float), m_colors);

	glDrawArrays(GL_LINES, 0, m_vertex_count);

	GLCheckGLError();

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);

	m_vertex_count = 0;
}

void GLLinesRenderer::AddLine(const b3Vec3& p1, const b3Vec3& p2, const b3Color& color)
{
	Vertex(p1.x, p1.y, p1.z, color.r, color.g, color.b, color.a);
	Vertex(p2.x, p2.y, p2.z, color.r, color.g, color.b, color.a);
}

void GLLinesRenderer::FlushLines(bool depthEnabled) 
{
	if (depthEnabled)
	{
		glEnable(GL_DEPTH_TEST);
	}
	else
	{
		glDisable(GL_DEPTH_TEST);
	}

	Flush();
}