Wheel, Hokuyo bracket and base-body descriptions (urdf/wheels, urdf/structures, urdf/inertia.urdf.xacro,
meshes/wheels, meshes/structures, meshes/bases/hls) are copied from
icclab/summit_xl_common summit_xl_description (humble, e248140), BSD licence, Robotnik Automation.
Changed: `$(find summit_xl_description)` → `$(find icclab_summit_xl)`, and in inertia.urdf.xacro the second
`solid_sphere_inertia` (the hollow sphere, it silently replaced the first) is now `hollow_sphere_inertia`.
Licence: LICENSE.summit_xl_common in this folder (Robotnik's original text).
