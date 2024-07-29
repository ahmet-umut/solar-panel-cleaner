import pysdf as psdf
if __name__ == "__main__":
	worl = psdf.SDF.from_file("wortem.sdf")

	p = psdf.base.Physics()
	p.type = "dart"
	p.real_time_factor = 2
	p.dart.solver.solver_type = "pgs"
	p.dart.collision_detector = "ode"

	worl.children[0].children[0] = p
	print(worl.children[0].children[0])
	worl.to_file("wor.sdf")
