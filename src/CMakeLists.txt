set(BOUNCE_HEADER_FILES
${BOUNCE_INCLUDE_DIR}/bounce/bounce.h

${BOUNCE_INCLUDE_DIR}/bounce/common/draw.h
${BOUNCE_INCLUDE_DIR}/bounce/common/types.h
${BOUNCE_INCLUDE_DIR}/bounce/common/settings.h
${BOUNCE_INCLUDE_DIR}/bounce/common/time.h
${BOUNCE_INCLUDE_DIR}/bounce/common/profiler.h
${BOUNCE_INCLUDE_DIR}/bounce/common/common.h

${BOUNCE_INCLUDE_DIR}/bounce/common/graphics/color.h
${BOUNCE_INCLUDE_DIR}/bounce/common/graphics/camera.h
${BOUNCE_INCLUDE_DIR}/bounce/common/graphics/debug_points.h
${BOUNCE_INCLUDE_DIR}/bounce/common/graphics/debug_lines.h
${BOUNCE_INCLUDE_DIR}/bounce/common/graphics/debug_triangles.h
${BOUNCE_INCLUDE_DIR}/bounce/common/graphics/debugdraw.h

${BOUNCE_INCLUDE_DIR}/bounce/common/math/math.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/vec2.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/vec3.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/vec4.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/mat22.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/mat33.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/mat44.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/quat.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/transform.h
${BOUNCE_INCLUDE_DIR}/bounce/common/math/sweep.h

${BOUNCE_INCLUDE_DIR}/bounce/common/memory/block_pool.h
${BOUNCE_INCLUDE_DIR}/bounce/common/memory/frame_allocator.h
${BOUNCE_INCLUDE_DIR}/bounce/common/memory/stack_allocator.h
${BOUNCE_INCLUDE_DIR}/bounce/common/memory/block_allocator.h

${BOUNCE_INCLUDE_DIR}/bounce/common/template/array.h
${BOUNCE_INCLUDE_DIR}/bounce/common/template/stack.h

${BOUNCE_INCLUDE_DIR}/bounce/collision/collision.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/clip.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/cluster.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/broad_phase.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/time_of_impact.h

${BOUNCE_INCLUDE_DIR}/bounce/collision/gjk/gjk.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/gjk/gjk_proxy.h

${BOUNCE_INCLUDE_DIR}/bounce/collision/sat/sat.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/sat/sat_hull_edge.h

${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/geometry.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/plane.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/ray.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/aabb.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/box_hull.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/capsule.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/cone_hull.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/cylinder_hull.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/grid_mesh.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/hull.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/mesh.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/sphere.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/geometry/triangle_hull.h

${BOUNCE_INCLUDE_DIR}/bounce/collision/shapes/sphere_shape.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/shapes/capsule_shape.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/shapes/hull_shape.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/shapes/mesh_shape.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/shapes/shape.h
${BOUNCE_INCLUDE_DIR}/bounce/collision/shapes/triangle_shape.h

${BOUNCE_INCLUDE_DIR}/bounce/collision/trees/dynamic_tree.h

${BOUNCE_INCLUDE_DIR}/bounce/dynamics/body.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/fixture.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contact_manager.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/island.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joint_manager.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/time_step.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/world.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/world_callbacks.h

${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/contact_solver.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/convex_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/mesh_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/sphere_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/capsule_sphere_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/capsule_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/triangle_sphere_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/triangle_capsule_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/triangle_hull_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/hull_sphere_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/hull_capsule_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/hull_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/mesh_sphere_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/mesh_capsule_contact.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/contacts/mesh_hull_contact.h

${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/cone_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/friction_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/joint_solver.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/motor_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/mouse_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/prismatic_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/revolute_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/sphere_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/spring_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/weld_joint.h
${BOUNCE_INCLUDE_DIR}/bounce/dynamics/joints/wheel_joint.h

${BOUNCE_INCLUDE_DIR}/bounce/rope/rope.h
${BOUNCE_INCLUDE_DIR}/bounce/rope/spatial.h
)

set(BOUNCE_SOURCE_FILES 	
	bounce/common/settings.cpp
	bounce/common/profiler.cpp
	
	bounce/common/graphics/graphics.cpp
	bounce/common/graphics/camera.cpp
	
	bounce/common/math/math.cpp

	bounce/common/memory/block_pool.cpp
	bounce/common/memory/frame_allocator.cpp
	bounce/common/memory/stack_allocator.cpp
	bounce/common/memory/block_allocator.cpp
	
	bounce/collision/broad_phase.cpp
	bounce/collision/collision.cpp
	bounce/collision/time_of_impact.cpp
	bounce/collision/clip.cpp
	bounce/collision/cluster.cpp
	
	bounce/collision/gjk/gjk.cpp
	bounce/collision/gjk/gjk_feature_pair.cpp

	bounce/collision/sat/sat.cpp
	bounce/collision/sat/sat_hull_edge.cpp

	bounce/collision/geometry/hull.cpp
	bounce/collision/geometry/mesh.cpp

	bounce/collision/shapes/capsule_shape.cpp
 	bounce/collision/shapes/hull_shape.cpp
	bounce/collision/shapes/mesh_shape.cpp
	bounce/collision/shapes/shape.cpp
	bounce/collision/shapes/sphere_shape.cpp
	bounce/collision/shapes/triangle_shape.cpp

	bounce/collision/trees/dynamic_tree.cpp

	bounce/collision/collide/collide_capsules.cpp
	bounce/collision/collide/collide_capsule_sphere.cpp
	bounce/collision/collide/collide_hull_capsule.cpp
	bounce/collision/collide/collide_hulls.cpp
	bounce/collision/collide/collide_hull_sphere.cpp
	bounce/collision/collide/collide_spheres.cpp
	bounce/collision/collide/collide_triangle_capsule.cpp
	bounce/collision/collide/collide_triangle_hull.cpp
	bounce/collision/collide/collide_triangle_sphere.cpp

	bounce/dynamics/body.cpp
	bounce/dynamics/fixture.cpp
	bounce/dynamics/contact_manager.cpp
	bounce/dynamics/contacts
	bounce/dynamics/island.cpp
	bounce/dynamics/joint_manager.cpp
	bounce/dynamics/world.cpp

	bounce/dynamics/contacts/contact.cpp
	bounce/dynamics/contacts/contact_solver.cpp
	bounce/dynamics/contacts/convex_contact.cpp
	bounce/dynamics/contacts/mesh_contact.cpp
	bounce/dynamics/contacts/sphere_contact.cpp
	bounce/dynamics/contacts/capsule_sphere_contact.cpp
	bounce/dynamics/contacts/capsule_contact.cpp
	bounce/dynamics/contacts/triangle_sphere_contact.cpp
	bounce/dynamics/contacts/triangle_capsule_contact.cpp
	bounce/dynamics/contacts/triangle_hull_contact.cpp
	bounce/dynamics/contacts/hull_sphere_contact.cpp
	bounce/dynamics/contacts/hull_capsule_contact.cpp
	bounce/dynamics/contacts/hull_contact.cpp
	bounce/dynamics/contacts/mesh_sphere_contact.cpp
	bounce/dynamics/contacts/mesh_capsule_contact.cpp
	bounce/dynamics/contacts/mesh_hull_contact.cpp

	bounce/dynamics/joints/cone_joint.cpp
	bounce/dynamics/joints/friction_joint.cpp
	bounce/dynamics/joints/joint.cpp
	bounce/dynamics/joints/joint_solver.cpp
	bounce/dynamics/joints/motor_joint.cpp
	bounce/dynamics/joints/mouse_joint.cpp
	bounce/dynamics/joints/prismatic_joint.cpp
	bounce/dynamics/joints/revolute_joint.cpp
	bounce/dynamics/joints/sphere_joint.cpp
	bounce/dynamics/joints/spring_joint.cpp
	bounce/dynamics/joints/weld_joint.cpp
	bounce/dynamics/joints/wheel_joint.cpp

	bounce/rope/rope.cpp
)

add_library(bounce STATIC ${BOUNCE_SOURCE_FILES} ${BOUNCE_HEADER_FILES})
target_include_directories(bounce PUBLIC ${BOUNCE_INCLUDE_DIR})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "src" FILES ${BOUNCE_SOURCE_FILES})
source_group(TREE ${BOUNCE_INCLUDE_DIR} PREFIX "include" FILES ${BOUNCE_HEADER_FILES})

set(QUICKHULL_HEADER_FILES ${BOUNCE_INCLUDE_DIR}/quickhull/quickhull.h)
set(QUICKHULL_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/quickhull/quickhull.c)

add_library(quickhull STATIC ${QUICKHULL_SOURCE_FILES} ${QUICKHULL_HEADER_FILES})
target_include_directories(quickhull PUBLIC ${BOUNCE_INCLUDE_DIR})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "src" FILES ${QUICKHULL_SOURCE_FILES})
source_group(TREE ${BOUNCE_INCLUDE_DIR} PREFIX "include" FILES ${QUICKHULL_HEADER_FILES})