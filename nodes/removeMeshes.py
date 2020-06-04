import trimesh
from trimesh.proximity import ProximityQuery
import numpy as np

class DistanceFieldCache:
    def __init__(self, mesh, binsize = 0.001, buffer = 0.002):
        self.Origin, self.Extent = mesh.bounding_box_oriented.bounds
        self.Origin = np.array(self.Origin)
        self.Extent = np.array(self.Extent)

        self.Origin -= buffer
        self.Extent += buffer
        self.NBins = np.array((self.Extent - self.Origin) // binsize, dtype=np.int)
        self.BinSize = binsize
        
        self.DistanceField = np.zeros(self.NBins)
        positions = []
        for i in range(self.DistanceField.size):
            ijk = np.unravel_index(i, self.NBins)
            positions.append((np.array(ijk) + 0.5) * binsize + self.Origin)
        positions = np.array(positions)
        distances = ProximityQuery(mesh).signed_distance(positions)
        
        for i in range(self.DistanceField.size):
            ijk = np.unravel_index(i, self.NBins)
            self.DistanceField[ijk] = distances[i]
    
    def distanceQuery(self, points):
        index = np.array((points - self.Origin) // self.BinSize, dtype=np.int)
        infMask = np.any((index < 0) | (index >= self.NBins), axis = 1)
        distances = np.full(len(points), -np.inf)
        for i in range(len(distances)):
            if not infMask[i]:
                distances[i] = self.DistanceField[tuple(index[i])]
        return distances

class MeshInstance():
    def __init__(self, name, position = np.zeros(3), rotation = np.eye(3)):
        self.Position = position
        self.Rotation = rotation
        self.Name = name

class PointCloudMeshFilter:
    def __init__(self, meshFiles, meshNames, minimumDistance = 0.002):

        self.MeshDistanceCache = {}
        for f,name in zip(meshFiles, meshNames):
            self.MeshDistanceCache[name] = DistanceFieldCache(trimesh.load_mesh(f), binsize = 0.001, buffer = minimumDistance)
        self.MinimumDistance = minimumDistance
        self.MeshInstances = []

    def setMeshInstances(self, instances):
        self.MeshInstances = instances

    def __call__(self, points):
        validMask = np.full(len(points), True, dtype = np.bool)
        for instance in self.MeshInstances:
            if instance.Name in self.MeshDistanceCache.keys():
                validMask = validMask & (self.distanceQuery(instance, points) < -self.MinimumDistance)
        return validMask

    def distanceQuery(self, meshInstance, points):
        print(points.dtype)
        print(meshInstance.Position)
        print(meshInstance.Rotation.T)
        points = np.matmul((points - meshInstance.Position), meshInstance.Rotation.T)
        return self.MeshDistanceCache[meshInstance.Name].distanceQuery(points)
    
    
