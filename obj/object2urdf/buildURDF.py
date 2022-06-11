from object2urdf import ObjectUrdfBuilder

object_folder = "../wall"

builder = ObjectUrdfBuilder(object_folder, urdf_prototype='_prototype_wall.urdf')
builder.build_urdf(filename="../wall/wall.obj", force_overwrite=True, decompose_concave=True, force_decompose=False, center = 'mass')