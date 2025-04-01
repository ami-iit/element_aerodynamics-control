from idyntree import bindings as idyntree
import pathlib
import open3d as o3d
import numpy as np
import toml


class Robot:
    def __init__(self, robot_name: str, urdf_path: str):
        self.name = robot_name
        self.urdf_path = urdf_path
        config_path = pathlib.Path(__file__).parents[1] / "config" / f"{self.name}.toml"
        self.config = toml.load(config_path)
        self._load_configuration()
        self.kindyn = self._load_kindyn()

    def _load_configuration(self):
        self.base_link = self.config["Model"]["Base_link"]
        self.joint_list = self.config["Joints"]["List"]
        self.joint_frames = self.config["Joints"]["Frames"]
        self.joint_limits = np.array(self.config["Joints"]["Limits"])
        # self.wind_tunnel_joint_limits = self.config["Joints"]["WindTunnelLimits"]
        self.collisions_frame_list = self.config["Collisions"]["FramesList"]
        self.collisions = {}
        start_index = 0
        for index, collisionFrame in enumerate(self.collisions_frame_list):
            self.collisions[collisionFrame] = {}
            self.collisions[collisionFrame]["Radii"] = np.array(
                self.config["Collisions"]["Radii"][index]
            )
            sphere_numbers = len(self.collisions[collisionFrame]["Radii"])
            self.collisions[collisionFrame]["Centers"] = np.array(
                self.config["Collisions"]["Centers"][
                    start_index : start_index + sphere_numbers
                ]
            )
            start_index += sphere_numbers

    def _load_kindyn(self):
        model_loader = idyntree.ModelLoader()
        reduced_model = model_loader.model()
        model_loader.loadReducedModelFromFile(str(self.urdf_path), self.joint_list)
        self.nDOF = reduced_model.getNrOfDOFs()
        kindyn = idyntree.KinDynComputations()
        kindyn.loadRobotModel(reduced_model)
        kindyn.setFloatingBase(self.base_link)
        return kindyn

    def set_state(self, pitch_angle, yaw_angle, joint_positions):
        w_H_b = idyntree.Transform(
            idyntree.Rotation.RotZ(-90).RotX(-pitch_angle).RotZ(yaw_angle).toNumPy(),
            [0.0, 0.0, 0.0],
        )
        zero_base_velocity = np.zeros(6)
        zero_joint_velocities = np.zeros_like(joint_positions)
        zero_gravity_acceleration = np.array([0, 0, 9.81])
        self.kindyn.setRobotState(
            w_H_b,
            joint_positions,
            zero_base_velocity,
            zero_joint_velocities,
            zero_gravity_acceleration,
        )
        return

    def get_free_floating_jacobian(self, frame_name):
        jac = idyntree.MatrixDynSize(6, self.nDOF + 6)
        self.kindyn.getFrameFreeFloatingJacobian(frame_name, jac)
        return jac.toNumPy()

    def is_colliding(self):
        collisions = {}
        coll_spheres = self._get_collision_spheres()
        # cycle on frame names that not contain the word turbine
        for frame_name, spheres in coll_spheres.items():
            if frame_name.find("turbine") == -1:
                collisions[frame_name] = False
                b_centers = spheres["Centers"]
                b_radii = spheres["Radii"]
                for b_center, b_radius in zip(b_centers, b_radii):
                    for frame_name2, spheres2 in coll_spheres.items():
                        if frame_name2.find("turbine") != -1:
                            j_centers = spheres2["Centers"]
                            j_radii = spheres2["Radii"]
                            for j_center, j_radius in zip(j_centers, j_radii):
                                # check if spheres are colliding
                                dist = np.linalg.norm(b_center - j_center)
                                if dist < (b_radius + j_radius):
                                    collisions[frame_name] = True
        return collisions

    def compute_collision_volume(self):
        coll_vol = {}
        coll_spheres = self._get_collision_spheres()
        # cycle on frame names that not contain the word turbine
        for frame_name, spheres in coll_spheres.items():
            if frame_name.find("turbine") == -1:
                b_centers = spheres["Centers"]
                b_radii = spheres["Radii"]
                volumes = np.zeros(len(b_centers))
                for idx, (b_c, b_r) in enumerate(zip(b_centers, b_radii)):
                    for frame_name2, spheres2 in coll_spheres.items():
                        if frame_name2.find("turbine") != -1:
                            j_centers = spheres2["Centers"]
                            j_radii = spheres2["Radii"]
                            for j_c, j_r in zip(j_centers, j_radii):
                                dist = np.linalg.norm(b_c - j_c)
                                vol = 1 - (dist - abs(b_r - j_r)) / (
                                    b_r + j_r - abs(b_r - j_r)
                                )
                                vol = max(vol, 0)
                                if vol > 1:
                                    vol = min(j_r / b_r, 1)
                                if vol > volumes[idx]:
                                    volumes[idx] = vol
                coll_vol[frame_name] = np.dot(volumes, b_radii) / np.sum(b_radii)
        return coll_vol

    # Visualization
    def visualize_with_collision_spheres(self, title, non_blocking):
        world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.2, origin=[0, 0, 0]
        )
        geometries = [{"name": "world_frame", "geometry": world_frame}]
        # Add robot meshes
        self._load_meshes()
        for mesh in self.meshes:
            geometries.append(
                {
                    "name": mesh["name"],
                    "geometry": mesh["mesh"],
                    "material": mesh["material"],
                }
            )
        # Add collision spheres
        coll_spheres = self._get_collision_spheres()
        for frame_name, spheres in coll_spheres.items():
            centers = spheres["Centers"]
            radii = spheres["Radii"]
            for idx, (center, radius) in enumerate(zip(centers, radii)):
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
                sphere.translate(center)
                sphere.compute_vertex_normals()
                sphere.compute_triangle_normals()
                sphere_mat = o3d.visualization.rendering.MaterialRecord()
                sphere_mat.shader = "defaultLitTransparency"
                sphere_mat.base_color = [1.0, 0.0, 0.0, 0.4]  # RGBA
                geometries.append(
                    {
                        "name": f"collision_{frame_name}_{idx}",
                        "geometry": sphere,
                        "material": sphere_mat,
                    }
                )
        o3d.visualization.draw(
            geometries,
            title=title,
            show_skybox=False,
            non_blocking_and_return_uid=non_blocking,
        )
        return

    def _load_meshes(self):
        self.links_list = []
        self.meshes_list = []
        self.link_H_geom = []
        model_loader = idyntree.ModelLoader()
        model_loader.loadModelFromFile(self.urdf_path)
        model = model_loader.model().copy()
        visuals = model.visualSolidShapes().getLinkSolidShapes()
        self.meshes = []
        for link_id in range(model.getNrOfLinks()):
            link_name = model.getLinkName(link_id)
            if visuals[link_id] == ():  # no visual
                continue
            link_visual = visuals[link_id][0]
            if link_visual.isExternalMesh():
                mesh_path = (
                    link_visual.asExternalMesh().getFileLocationOnLocalFileSystem()
                )
                if mesh_path.endswith(".obj") or mesh_path.endswith(".stl"):
                    self._get_mesh(link_name, link_visual, mesh_path)
                else:
                    raise ValueError(
                        f"Extension {mesh_path.split('.')[-1]} not supported"
                    )
        return

    def _get_mesh(self, link_name, link_visual, mesh_path):
        link_geometry = o3d.io.read_triangle_model(mesh_path)
        scale = link_visual.asExternalMesh().getScale()[0]
        w_H_l = self.kindyn.getWorldTransform(link_name)
        l_H_g = link_visual.getLink_H_geometry()
        w_H_g = w_H_l * l_H_g
        for mesh_id, mesh_info in enumerate(link_geometry.meshes):
            mesh = mesh_info.mesh
            mesh.scale(scale, center=[0, 0, 0])
            mesh.transform(w_H_g.asHomogeneousTransform().toNumPy())
            mesh.compute_vertex_normals()
            mesh.compute_triangle_normals()
            material = link_geometry.materials[mesh_info.material_idx]
            if mesh_path.endswith(".stl"):
                material.base_color = link_visual.getMaterial().color().toNumPy()
            self.meshes.append(
                {"name": f"{link_name}_{mesh_id}", "mesh": mesh, "material": material}
            )
        return

    def _get_collision_spheres(self):
        collision_spheres = {}
        for frame_i in self.collisions_frame_list:
            w_H_i = (
                self.kindyn.getWorldTransform(frame_i)
                .asHomogeneousTransform()
                .toNumPy()
            )
            b_centers_i = self.collisions[frame_i]["Centers"]
            centers_i = self._compute_self_collision_centers(w_H_i, b_centers_i)
            radiuses_i = self.collisions[frame_i]["Radii"]
            collision_spheres[frame_i] = {
                "Centers": centers_i,
                "Radii": radiuses_i,
            }
        return collision_spheres

    def _compute_self_collision_centers(self, w_H_i, centers_b):
        w_R_i = w_H_i[0:3, 0:3]
        w_o_i = w_H_i[0:3, 3]
        centers_w = np.zeros(centers_b.shape)
        for i in range(centers_b.shape[0]):
            centers_w[i, :] = w_o_i + np.dot(w_R_i, centers_b[i, :])
        return centers_w
