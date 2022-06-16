from object2urdf import ObjectUrdfBuilder

object_folder = "../obstacles"

builder = ObjectUrdfBuilder(object_folder, urdf_prototype='_prototype_obstacle.urdf')
builder.build_urdf(filename="../obstacles/obstacle2.obj", force_overwrite=True, decompose_concave=True, force_decompose=False, center = 'mass')