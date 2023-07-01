
import os
import cv2
import numpy as np
import open3d as o3d
import copy
# Load the 3D model
# Replace 'model_path' with the actual path to your 3D model file* 
# Attempt with obj file
model_path = "./obj/ETHmodel.obj"
mesh = o3d.io.read_triangle_mesh(model_path,enable_post_processing=True)
# model_path = os.path.expanduser("~")+"/Downloads/Neuheim_field_test_post_processed_RGB_colored_mesh.ply"

# o3d.visualization.draw_geometries([mesh])

points = np.array(mesh.vertices)
indices = np.array(mesh.triangles)
colors = np.array(mesh.vertex_colors)



# load a demo obj file
# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000)
# mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1000)

# Load the camera intrinsic parameters
# Replace 'intrinsic_path' with the actual path to your camera intrinsic file
intrinsic_matrix = np.array([[400, 0, 320], [0, 400, 240], [0, 0, 1]])

# Set the viewpoint pose (camera pose)
# Replace 'viewpoint_pose' with the actual pose you want to querya
# It should be a 4x4 transformation matrix representing the camera pose in the world coordinate system
extrinsic_matrix = np.eye(4)
extrinsic_matrix[0:3, 3] = np.array([100, 100, 100])


# Render the scene from the given viewpoint
# Specify the desired image dimensions, e.g., (width, height)
image_size = (1920, 1080)
image = np.zeros((image_size[1], image_size[0], 3), dtype=np.uint8)  # Placeholder image, replace with your actual rendering process

correspondence_pxs = [(100, 100), (0, 2)]
# Perform raycasting and retrieve the 3D points and corresponding 2D pixels
# Replace 'correspondence_pxs' with the actual pixel coordinates you want to query
correspondence_3d_points = np.zeros((len(correspondence_pxs), 3))
correspondence_2d_pixels = np.zeros((len(correspondence_pxs), 2))
for idx, px in enumerate(correspondence_pxs):
    # Perform raycasting
    pass


# visualize the mesh in open3d
# vis = o3d.visualization.Visualizer()
# vis.create_window()
# #include axes
# vis.add_geometry(mesh)
# vis.run()
# vis.destroy_window()

# Create a renderer with the desired image size
img_width = image_size[0]
img_height = image_size[1]
render = o3d.visualization.rendering.OffscreenRenderer(img_width, img_height)

# Pick a background colour (default is light gray)
render.scene.set_background([0.1, 0.2, 0.3, 1.0])  # RGBA

# Create the mesh geometry.
# (We use arrows instead of a sphere, to show the rotation more clearly.)
# mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=np.array([0.0, 0.0, 0.0]))

# Show the original coordinate axes for comparison.
# X is red, Y is green and Z is blue.
render.scene.show_axes(True)

# Define a simple unlit Material.
# (The base color does not replace the arrows' own colors.)
mtl = o3d.visualization.rendering.MaterialRecord()#, for later versions of Open3D
mtl.base_color = [1.0, 1.0, 1.0, 1.0]  # RGBA
mtl.shader = "defaultUnlit"

# Add the arrow mesh to the scene.
# (These are thicker than the main axis arrows, but the same length.)
# render.scene.add_geometry("odel", mesh, mtl)
render.scene.add_model("model", mesh)

# Since the arrow material is unlit, it is not necessary to change the scene lighting.
#render.scene.scene.enable_sun_light(False)
#render.scene.set_lighting(render.scene.LightingProfile.NO_SHADOWS, (0, 0, 0))

# Optionally set the camera field of view (to zoom in a bit)
vertical_field_of_view = 45.0  # between 5 and 90 degrees
aspect_ratio = img_width / img_height  # azimuth over elevation
near_plane = 0.1
far_plane = 50.0
fov_type = o3d.visualization.rendering.Camera.FovType.Vertical
render.scene.camera.set_projection(vertical_field_of_view, aspect_ratio, near_plane, far_plane, fov_type)

# Look at the origin from the front (along the -Z direction, into the screen), with Y as Up.
center = [10, 10, 0]  # look_at target
eye = [10, 10, 50]  # camera position
up = [0, 1, 0]  # camera orientation
render.scene.camera.look_at(center, eye, up)

# Read the image into a variable
img_o3d = render.render_to_image()

# Display the image in a separate window
# (Note: OpenCV expects the color in BGR format, so swop red and blue.)
img_cv2 = cv2.cvtColor(np.array(img_o3d), cv2.COLOR_RGBA2BGRA)
# cv2.imshow("Preview window", img_cv2)
# cv2.waitKey()

# # Optionally write it to a PNG file
cv2.imwrite("a_rendered_image.png", img_cv2)
