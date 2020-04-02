def my_conversion(partent, child):
	partent=bpy.data.objects[partent]
	child=bpy.data.objects[child]
	Twp=partent.matrix_world.copy()
	Twc=child.matrix_world.copy()
	Tpw = partent.copy()
	Twp.invert()
	return Tpw*Twc