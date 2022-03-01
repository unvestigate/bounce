set(TESTBED_SOURCE_FILES
	framework/open_gl/gl_shader.h
	framework/open_gl/gl_shader.cpp
	framework/open_gl/gl_points_renderer.h
	framework/open_gl/gl_points_renderer.cpp
	framework/open_gl/gl_lines_renderer.h
	framework/open_gl/gl_lines_renderer.cpp
	framework/open_gl/gl_triangles_renderer.h
	framework/open_gl/gl_triangles_renderer.cpp
	framework/imgui/imgui_impl_glfw.cpp
	framework/imgui/imgui_impl_glfw.h
	framework/imgui/imgui_impl_opengl3.cpp
	framework/imgui/imgui_impl_opengl3.h
	framework/imgui/imgui_impl_opengl3_loader.h
	framework/body_dragger.cpp
	framework/body_dragger.h
	framework/draw.h
	framework/main.cpp
	framework/model.cpp
	framework/model.h
	framework/test.cpp
	framework/test.h
	framework/view.cpp
	framework/view.h
	framework/view_model.cpp
	framework/view_model.h

	tests/rope_test.h
	tests/aabb_time_of_impact.h
	tests/angular_motion.h
	tests/body_types.h
	tests/box_edge_contact.h
	tests/box_face_contact.h
	tests/box_stack.h
	tests/capsule_collision.h
	tests/capsule_spin.h
	tests/capsule_stack.h
	tests/cluster.h
	tests/compound_body.h
	tests/cone_test.h
	tests/convex_hull.h
	tests/conveyor_belt.h
	tests/deep_capsule.h
	tests/distance_test.h
	tests/gyro_motion.h
	tests/hinge_chain.h
	tests/hull_collision.h
	tests/hull_contact_test.h
	tests/initial_overlap.h
	tests/jenga.h
	tests/linear_motion.h
	tests/linear_time_of_impact.h
	tests/mesh_contact_test.h
	tests/motor_test.h
	tests/multiple_pendulum.h
	tests/newton_cradle.h
	tests/prismatic_test.h
	tests/pyramid.h
	tests/pyramids.h
	tests/quadric_shapes.h
	tests/ragdoll.h
	tests/ray_cast.h
	tests/revolute_test.h
	tests/sensor_test.h
	tests/shape_cast.h
	tests/shape_stack.h
	tests/sheet_stack.h
	tests/sphere_stack.h
	tests/spring_test.h
	tests/time_of_impact.h
	tests/triangle_contact_test.h
	tests/tumbler.h
	tests/varying_friction.h
	tests/varying_restitution.h
	tests/weld_test.h
	tests/wheel_test.h
        tests/shift_center.h
)

add_executable(testbed ${TESTBED_SOURCE_FILES})
target_include_directories(testbed PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${BOUNCE_EXTERNAL_DIR} ${BOUNCE_EXAMPLES_DIR} ${BOUNCE_INCLUDE_DIR})
target_link_libraries(testbed PUBLIC glad glfw imgui quickhull bounce)
set_target_properties(testbed PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)

# message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")

# Copy files (if needed)
#add_custom_command(
#        TARGET testbed POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_directory
#                ${CMAKE_CURRENT_SOURCE_DIR}/data/
#                ${CMAKE_CURRENT_BINARY_DIR}/data/)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${TESTBED_SOURCE_FILES})
