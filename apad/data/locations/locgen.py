import random

lat=random.uniform(0,30)
lon=random.uniform(0,30)

file = open("swarm_loc.yaml", "w")
file.write("#Swarm location information\n")
file.write("LocalOriginLat: "+str(lat)+ "\n")
file.write("LocalOriginLon: "+str(lon)+ "\n")
file.close()


