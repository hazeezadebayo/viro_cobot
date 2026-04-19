import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R, Slerp



import csv
import numpy as np

def read_poses_from_csv(filename):
    """
    Read poses from a CSV file.
    Each row should contain: x, y, z, qx, qy, qz, qw
    """
    poses = []
    with open(filename, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            pose = [
                float(row['y1']),  # x
                float(row['y2']),  # y
                float(row['y3']),  # z
                float(row['y4']), # qx
                float(row['y5']), # qy
                float(row['y6']), # qz
                float(row['y7'])  # qw
            ]
            poses.append(pose)
    return poses

def write_trajectory_to_csv(trajectory, filename):
    """
    Write trajectory data to a CSV file.
    Each row includes: time, px, py, pz, qx, qy, qz, qw, vx, vy, vz, ax, ay, az
    """
    with open(filename, mode='w', newline='') as csvfile:
        fieldnames = [
            'time',
            'px', 'py', 'pz',
            'qx', 'qy', 'qz', 'qw',
            'vx', 'vy', 'vz',
            'ax', 'ay', 'az'
        ]
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        for point in trajectory:
            writer.writerow({
                'time': point['time'],
                'px': point['position'][0],
                'py': point['position'][1],
                'pz': point['position'][2],
                'qx': point['orientation'][0],
                'qy': point['orientation'][1],
                'qz': point['orientation'][2],
                'qw': point['orientation'][3],
                'vx': point['velocity'][0],
                'vy': point['velocity'][1],
                'vz': point['velocity'][2],
                'ax': point['acceleration'][0],
                'ay': point['acceleration'][1],
                'az': point['acceleration'][2]
            })

def generate_trajectory(
        poses,
        velocity=None,
        acceleration=None,
        dt=0.01,
        show_plot=False
    ):
    """
    Generate a time-parameterized trajectory from a sequence of poses.
    Each pose is [x, y, z, qx, qy, qz, qw].
    
    If 'velocity' is None, we do a simple linear interpolation (like "pos only").
    If 'velocity' is given but 'acceleration' is None, we do a cubic interpolation (pos + vel).
    If both 'velocity' and 'acceleration' are given, we do a quintic interpolation (pos + vel + acc).
    
    If show_plot=True, it will plot the position, velocity, and acceleration
    over time for the chosen method.
    
    Returns:
      trajectory: a list of dictionaries, each with:
         - time
         - position (3D)
         - orientation (quaternion)
         - velocity (3D)
         - acceleration (3D)
    """
    poses = np.array(poses)
    n = poses.shape[0]
    if n < 2:
        raise ValueError("Need at least 2 poses to generate a trajectory!")
    
    # Determine which method to use based on velocity/acceleration
    if velocity is None:
        method = "linear"
    elif acceleration is None:
        method = "cubic"
    else:
        method = "quintic"
    
    # Storage for final trajectory
    trajectory = []
    current_time = 0.0
    
    # For plotting (if requested)
    all_times = []
    all_positions = []
    all_velocities = []
    all_accelerations = []
    
    def distance_3d(a, b):
        return np.linalg.norm(b - a)
    
    # Helper: Compute time for a segment given distance.
    def compute_segment_time(dist, v, acc):
        if v is None or v <= 0:
            return 0.0
        if acc is None or acc <= 0:
            return dist / v
        d_min = v**2 / acc
        if dist >= d_min:
            t_acc = v / acc
            d_acc = 0.5 * acc * t_acc**2
            d_decel = d_acc
            d_cruise = dist - (d_acc + d_decel)
            t_cruise = d_cruise / v if v > 0 else 0.0
            return t_acc + t_cruise + t_acc
        else:
            return 2 * np.sqrt(dist / acc)
    
    # Helper: Compute cubic polynomial coefficients for 1D.
    def compute_cubic_coeffs(p0, v0, p1, v1, T):
        A = np.array([
            [1, 0,        0,          0],
            [0, 1,        0,          0],
            [1, T,      T**2,      T**3],
            [0, 1,      2*T,    3*(T**2)]
        ], dtype=float)
        b = np.array([p0, v0, p1, v1], dtype=float)
        a0, a1, a2, a3 = np.linalg.solve(A, b)
        return np.array([a0, a1, a2, a3])
    
    def eval_cubic(coeffs, t):
        pos = coeffs[0] + coeffs[1]*t + coeffs[2]*(t**2) + coeffs[3]*(t**3)
        vel = coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*(t**2)
        acc = 2*coeffs[2] + 6*coeffs[3]*t
        return pos, vel, acc
    
    # Helper: Compute quintic polynomial coefficients for 1D.
    def compute_quintic_coeffs(p0, v0, a0, p1, v1, a1, T):
        M = np.array([
            [   T**3,     T**4,     T**5],
            [ 3*T**2,   4*T**3,   5*T**4],
            [   6*T,   12*T**2,  20*T**3]
        ], dtype=float)
        rhs = np.array([
            p1 - (p0 + v0*T + 0.5*a0*T**2),
            v1 - (v0 + a0*T),
            a1 - a0
        ], dtype=float)
        x = np.linalg.solve(M, rhs)
        return np.array([p0, v0, 0.5*a0, x[0], x[1], x[2]], dtype=float)
    
    def eval_quintic(coeffs, t):
        powers = np.array([1, t, t**2, t**3, t**4, t**5], dtype=float)
        p = np.dot(coeffs, powers)
        dcoeffs = np.array([
            coeffs[1],
            2*coeffs[2],
            3*coeffs[3],
            4*coeffs[4],
            5*coeffs[5]
        ], dtype=float)
        v = np.polyval(dcoeffs[::-1], t)
        ddcoeffs = np.array([
            2*coeffs[2],
            6*coeffs[3],
            12*coeffs[4],
            20*coeffs[5]
        ], dtype=float)
        a = np.polyval(ddcoeffs[::-1], t)
        return p, v, a

    # Build trajectory by segment.
    for i in range(n - 1):
        pA = poses[i, :3]
        pB = poses[i+1, :3]
        qA = poses[i, 3:]
        qB = poses[i+1, 3:]
        
        dist = distance_3d(pA, pB)
        
        # Determine duration T for this segment.
        if method == "linear":
            v_temp = 0.1 if velocity is None else velocity
            T = dist / v_temp if v_temp > 1e-9 else 0.0
        elif method == "cubic":
            v_temp = velocity if velocity else 0.1
            T = dist / v_temp
        else:  # quintic
            v_temp = velocity if velocity else 0.1
            acc_temp = acceleration if acceleration else 0.1
            T = compute_segment_time(dist, v_temp, acc_temp)
        
        num_samples = int(np.ceil(T / dt)) + 1 if T > 1e-9 else 1
        times_local = np.linspace(0, T, num_samples)
        
        # Orientation interpolation: use Slerp.
        rA = R.from_quat(qA)
        rB = R.from_quat(qB)
        key_times = [0, T]
        slerp_obj = Slerp(key_times, R.from_quat([qA, qB]))
        
        # Precompute polynomial coefficients per dimension.
        if method == "linear":
            pass  # no polynomial needed
        elif method == "cubic":
            cubic_coeffs = []
            for dim in range(3):
                c = compute_cubic_coeffs(pA[dim], 0, pB[dim], 0, T)
                cubic_coeffs.append(c)
        else:  # quintic
            quintic_coeffs = []
            for dim in range(3):
                c = compute_quintic_coeffs(pA[dim], 0, 0, pB[dim], 0, 0, T)
                quintic_coeffs.append(c)
        
        # Sample the segment.
        for tlocal in times_local:
            if method == "linear":
                param = tlocal / T if T > 1e-9 else 0
                pos = pA + (pB - pA) * param
                vel = (pB - pA) / T if T > 1e-9 else np.zeros(3)
                acc_vec = np.zeros(3)
            elif method == "cubic":
                pos = np.zeros(3)
                vel = np.zeros(3)
                acc_vec = np.zeros(3)
                for dim in range(3):
                    p_val, v_val, a_val = eval_cubic(cubic_coeffs[dim], tlocal)
                    pos[dim] = p_val
                    vel[dim] = v_val
                    acc_vec[dim] = a_val
            else:  # quintic
                pos = np.zeros(3)
                vel = np.zeros(3)
                acc_vec = np.zeros(3)
                for dim in range(3):
                    p_val, v_val, a_val = eval_quintic(quintic_coeffs[dim], tlocal)
                    pos[dim] = p_val
                    vel[dim] = v_val
                    acc_vec[dim] = a_val
            
            # Interpolate orientation using Slerp.
            quat = slerp_obj([tlocal]).as_quat()[0]
            
            time_global = current_time + tlocal
            waypoint = {
                "time": time_global,
                "position": pos,
                "orientation": quat,
                "velocity": vel,
                "acceleration": acc_vec
            }
            trajectory.append(waypoint)
            
            if show_plot:
                all_times.append(time_global)
                all_positions.append(pos[0])    # Track X for demonstration
                all_velocities.append(vel[0])
                all_accelerations.append(acc_vec[0])
        
        current_time += T
    
    # Append final pose exactly.
    final_pose = poses[-1]
    trajectory.append({
        "time": current_time,
        "position": final_pose[:3],
        "orientation": final_pose[3:],
        "velocity": np.zeros(3),
        "acceleration": np.zeros(3)
    })
    if show_plot:
        all_times.append(current_time)
        all_positions.append(final_pose[0])
        all_velocities.append(0.0)
        all_accelerations.append(0.0)
    
    if show_plot:
        fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        axs[0].plot(all_times, all_positions, label=f'{method.title()} X-position')
        axs[0].set_ylabel('Position (X)')
        axs[0].grid(True)
        axs[0].legend()
        
        axs[1].plot(all_times, all_velocities, label=f'{method.title()} X-velocity')
        axs[1].set_ylabel('Velocity (X)')
        axs[1].grid(True)
        axs[1].legend()
        
        axs[2].plot(all_times, all_accelerations, label=f'{method.title()} X-acc')
        axs[2].set_ylabel('Acceleration (X)')
        axs[2].set_xlabel('Time (s)')
        axs[2].grid(True)
        axs[2].legend()
        
        plt.tight_layout()
        plt.show()
    
    return trajectory


def process_csv_to_trajectory(input_csv, output_csv, velocity=None, acceleration=None, dt=0.01, show_plot=False):
    poses = read_poses_from_csv(input_csv)
    trajectory = generate_trajectory(
        poses=poses,
        velocity=velocity,
        acceleration=acceleration,
        dt=dt,
        show_plot=show_plot
    )
    write_trajectory_to_csv(trajectory, output_csv)

# -----------------------
# Example usage:
if __name__ == "__main__":

    # sample usage:
    # expected input type
    poses = np.array([
        [0, 0, 0, 0, 0, 0, 1],
        [0.2, 0.0, 0.0, 0, 0, 0.7071068, 0.7071068],
        [0.4, 0.1, 0.0, 0, 0, 0.9238795, 0.3826834],
        [0.7, 0.1, 0.2, 0, 0, 0.3826834, 0.9238795],
        [1.0, 0.2, 0.3, 0, 0, 0, 1]
    ])
    
    # 1) No velocity => "linear"
    traj_linear = generate_trajectory(poses, velocity=None, acceleration=None, dt=0.01, show_plot=True)
    
    # 2) Velocity but no acceleration => "cubic"
    # Set desired velocity (e.g., 0.1 units/sec) and optional acceleration (e.g., 0.05 units/sec^2)
    v = 0.2
    traj_cubic = generate_trajectory(poses, velocity=v, acceleration=None, dt=0.01, show_plot=True)
    
    # 3) Velocity + acceleration => "quintic"
    a = 0.1
    traj_quintic = generate_trajectory(poses, velocity=v, acceleration=a, dt=0.01, show_plot=True)
    
    # Print the trajectory times and positions for the quintic case.
    for pt in traj_quintic:
        print(f"t={pt['time']:.3f} s, pos={pt['position']}, quat={pt['orientation']}")

    # real usage:
    # get the generated surface path and then use it to generate a trajectory. output the result to a csv.
    pkg_path = r"C:\Users\MCFLY\Downloads\mcfly_ros2_common-main-mcfly_scan_n_plan\mcfly_ros1_s_n_p\mcfly_scan_n_plan"
    csv_input_file_path = pkg_path + "/config/scan_n_plan_path.csv"
    csv_output_file_path = pkg_path + "/config/scan_n_plan_traj.csv"
    process_csv_to_trajectory(
        input_csv=csv_input_file_path,
        output_csv=csv_output_file_path,
        velocity=0.1,
        acceleration=0.05,
        dt=0.02,
        show_plot=True
    )

