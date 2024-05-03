import time
import json
from some_robotics_library import Robot, Sphere, PoseOnSphere, Joint

class DataGenerator:
    def __init__(self, filename):
        self.filename = filename
        self.robot = Robot("manipulator")
        self.ik_data = self.load_data(self.filename)
        self.ik_data_result = {
            'resolution': self.ik_data['resolution'],
            'radius': self.ik_data['radius'],
            'sphere_sample': self.ik_data['sphere_sample'],
            'spheres': []
        }

    def load_data(self, filename):
        with open(filename, 'r') as file:
            return json.load(file)

    def data_comparator(self):
        for s in self.ik_data['spheres']:
            sphere_result = Sphere(s['x'], s['y'], s['z'])
            for pose_on_sphere in s['poses']:
                pose_on_sphere_result = PoseOnSphere(
                    pose_on_sphere['x'], pose_on_sphere['y'], pose_on_sphere['z'],
                    pose_on_sphere['theta_x'], pose_on_sphere['theta_y'], pose_on_sphere['theta_z'],
                    pose_on_sphere['theta_w']
                )

                if 'joints' in pose_on_sphere:
                    for joint_data in pose_on_sphere['joints']:
                        joint = Joint.from_vector(joint_data)
                        if not self.robot.check_collision(joint.to_vector()):
                            pose_on_sphere_result.joints.append(joint)
                            break
                        else:
                            print("Joint move failed")
                sphere_result.add_pose(pose_on_sphere_result)
            self.ik_data_result['spheres'].append(sphere_result)

    def write_data(self):
        with open('/home/will/master_ik_data_result.json', 'w') as file:
            json.dump(self.ik_data_result, file, indent=2)

if __name__ == "__main__":
    data_generator = DataGenerator("/home/will/master_ik_data.json")
    time.sleep(2)
    start_time = time.time()
    data_generator.data_comparator()
    data_generator.write_data()
    end_time = time.time()
    duration = end_time - start_time
    print(f"Total time taken: {duration:.2f} seconds")
    print("Data generation done")
