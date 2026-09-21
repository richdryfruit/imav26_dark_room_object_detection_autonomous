"""
PX4 SITL + Gazebo (gz sim 8) for mission_fsm_part1/2, on a laptop.

    ros2 launch drone_testing sitl.launch.py sitl_src:=<imav_indoor_2026_sitl checkout>

World: imav2026_indoor_v9, REAL scale, patched into /tmp to the MEASURED
course (it ships without PX4's sensor systems and with a different window):
    takeoff pad   id 0   (-2.75, -6.47)    <- default spawn (part 1)
    window marker id 2   (-2.75,  2.83)    9.30 m ahead, 0.40 m marker
                                           <- part 2 spawn: y:=2.83
    blue opening         0.60 x 0.60 m, centre (-2.90, 4.50, z 1.90):
                         0.15 m LEFT of the marker, 1.67 m beyond it
    red opening          0.50 x 0.50 m, centre (-1.50, 4.50, z 2.00)
    dark room            2.5 x 2.5 x 2.5 m, south wall y 4.50
    dolls                3 single dolls cut from MODERN_DOLL_FAMILY

world:=imav2026_scaled still works (x2.2, window at 3.85 m -- too high for
part 2's 1.9 m flight; pass x:=-4.4 y:=-14.3 marker_size:=0.88).

Starts: gz sim on the imav_indoor_2026 world, the x500 with a down camera,
PX4 SITL (standalone, attaches to the spawned model), the uXRCE-DDS agent
(UDP 8888), the ROS<->gz bridge and aruco_pose reading the sim down camera.
The flight node is run by hand in a second terminal, as on the real drone.

ISOLATION -- READ THIS. The sim runs in its own ROS domain (ros_domain, default
42) with discovery limited to this machine, and PX4 SITL's DDS client is put in
the same domain (UXRCE_DDS_DOM_ID). On a shared Wi-Fi, domain 0 can carry a REAL
vehicle's /fmu topics; a sim node there reads the real vehicle's state and
sends it commands. The terminal that runs the mission node MUST export the same:

    export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

SENSORS AND ESTIMATOR, matched to the aircraft rather than PX4's SITL default:
    x/y     PMW3901 optical flow (the sim flow camera is its 42 deg FOV)
    height  baro reference + TFmini Plus (conditional): the sim's single-beam
            lidar clipped to 0.1-12 m, 2 cm noise. Range as the height REF
            stops EKF2 ever starting flow fusion -- see params below.
    GPS     simulated but NOT fused (EKF2_GPS_CTRL 0)

Gazebo's yaw 0 is +X; the course runs along +Y, hence yaw 1.5708 (facing
the window marker from the takeoff pad).
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Systems PX4's gz_bridge needs sensor data from, and the origin navsat needs.
_SIM_MODELS = '/tmp/imav_sim_models'       # generated models (dolls)
_WORLD_SYSTEMS = [
    ('gz-sim-air-pressure-system', 'gz::sim::systems::AirPressure'),
    ('gz-sim-magnetometer-system', 'gz::sim::systems::Magnetometer'),
    ('gz-sim-navsat-system', 'gz::sim::systems::NavSat'),
    ('gz-sim-imu-system', 'gz::sim::systems::Imu'),
    ('libOpticalFlowSystem.so', 'custom::OpticalFlowSystem'),
]
_ORIGIN = ('<spherical_coordinates><surface_model>EARTH_WGS84</surface_model>'
           '<latitude_deg>47.397742</latitude_deg><longitude_deg>8.545594'
           '</longitude_deg><elevation>488.0</elevation><heading_deg>0'
           '</heading_deg></spherical_coordinates>')


def _dim(sdf, scale):
    """Scale every light's diffuse/specular and the scene ambient by scale."""
    import re
    if abs(scale - 1.0) < 1e-6:
        return sdf

    def mul(m):
        vals = m.group(2).split()
        rgb = [f"{float(v) * scale:.3f}" for v in vals[:3]]
        return f"<{m.group(1)}>{' '.join(rgb + vals[3:])}</{m.group(1)}>"

    def in_light(m):
        return re.sub(r'<(diffuse|specular)>([^<]*)</\1>', mul, m.group(0))

    sdf = re.sub(r'<light\b.*?</light>', in_light, sdf, flags=re.S)
    return re.sub(r'<scene>.*?</scene>',
                  lambda m: re.sub(r'<(ambient)>([^<]*)</\1>', mul, m.group(0)),
                  sdf, flags=re.S)


# ---- the real course, measured (imav2026_indoor_v9 only) -----------------
# Dark room: 2.5 m cube, south (window) wall at y 4.50, x -3.50 (left, seen
# from outside) .. -1.00 (right), 0.04 m thick. Openings are the MEASURED
# openings; the colour is a 2 cm band on the outside face around each one.
_WALL_Y, _WALL_L, _WALL_R, _WALL_H, _WALL_T = 4.50, -3.50, -1.00, 2.50, 0.04
_BAND = 0.02


def _course(scale=1.0):
    """Blue/red openings, marker and pad for a blue opening scaled by scale.

    scale 1: as measured -- blue 0.60 m, centre 0.60 m from the left edge and
    1.90 m up. Larger: the blue opening grows about its centre, then is pushed
    back inside the wall (5 cm margin), and red moves right to stay clear.
    The marker stays 1.67 m out and 0.15 m right of the blue axis.
    """
    w = 0.60 * scale
    cx = min(max(0.60, 0.05 + w / 2), 2.45 - w / 2)
    cz = min(max(1.90, 0.05 + w / 2), _WALL_H - 0.05 - w / 2)
    blue = (_WALL_L + cx - w / 2, _WALL_L + cx + w / 2, cz - w / 2, cz + w / 2)
    red_x0 = max(1.75, cx + w / 2 + 0.10)
    red = (_WALL_L + red_x0, _WALL_L + min(red_x0 + 0.50, 2.45), 1.75, 2.25)
    marker = (_WALL_L + cx + 0.15, _WALL_Y - 1.67)
    pad = (marker[0], marker[1] - 9.30)
    return blue, red, marker, pad
# Three single dolls cut from MODERN_DOLL_FAMILY (4 dolls, 4 mesh pieces),
# spread across the room floor, clear of the walls.
_DOLLS = [((-3.10, 6.60, 1.0), 0), ((-1.40, 6.50, -2.0), 1), ((-1.45, 5.05, 2.6), 2)]


def _box(name, x0, x1, z0, z1, y, t, rgb):
    cx, cz, sx, sz = (x0 + x1) / 2, (z0 + z1) / 2, x1 - x0, z1 - z0
    col = f'{rgb} 1'
    return (f'<model name="{name}"><static>true</static><pose>{cx:.4f} {y:.4f} '
            f'{cz:.4f} 0 0 0</pose><link name="link"><collision name="col">'
            f'<geometry><box><size>{sx:.4f} {t:.4f} {sz:.4f}</size></box>'
            f'</geometry></collision><visual name="v"><geometry><box><size>'
            f'{sx:.4f} {t:.4f} {sz:.4f}</size></box></geometry><material>'
            f'<ambient>{col}</ambient><diffuse>{col}</diffuse></material>'
            '</visual></link></model>')


def _south_wall(b, r):
    """The wall as boxes around the two openings, plus the colour bands."""
    L, R, T, H, y = _WALL_L, _WALL_R, _WALL_T, _WALL_H, _WALL_Y
    wall = '0.60 0.50 0.38'
    zlo, zhi = min(b[2], r[2]), max(b[3], r[3])
    parts = [
        ('room_S_bot', L, R, 0.0, zlo), ('room_S_top', L, R, zhi, H),
        ('room_S_a', L, b[0], zlo, zhi), ('room_S_b', b[1], r[0], zlo, zhi),
        ('room_S_c', r[1], R, zlo, zhi),
    ]
    for tag, (x0, x1, z0, z1) in (('blue', b), ('red', r)):
        if z0 > zlo:
            parts.append((f'room_S_under_{tag}', x0, x1, zlo, z0))
        if z1 < zhi:
            parts.append((f'room_S_over_{tag}', x0, x1, z1, zhi))
    out = [_box(n, x0, x1, z0, z1, y, T, wall) for n, x0, x1, z0, z1 in parts]
    yb = y - T / 2 - 0.004                      # just proud of the outside face
    for tag, (x0, x1, z0, z1), rgb in (('blue', b, '0.0 0.3 1.0'),
                                       ('red', r, '1.0 0.0 0.0')):
        e = _BAND
        for side, box in (('top', (x0 - e, x1 + e, z1, z1 + e)),
                          ('bot', (x0 - e, x1 + e, z0 - e, z0)),
                          ('L', (x0 - e, x0, z0, z1)), ('R', (x1, x1 + e, z0, z1))):
            out.append(_box(f'room_{tag}_{side}', *box, yb, 0.006, rgb))
    return '\n'.join(out)


def _doll_models(models_dir, out_dir, scale=5):
    """Split MODERN_DOLL_FAMILY into single-doll models under out_dir.

    scale is the mesh scale: 5 for the real-scale world, 10 for
    imav2026_scaled (what its MODERN_DOLL_FAMILY_SCALED uses).
    """
    tag = f'IMAV_DOLL_S{scale}_'
    src = os.path.join(models_dir, 'MODERN_DOLL_FAMILY')
    obj = os.path.join(src, 'meshes', 'model.obj')
    tex = os.path.join(src, 'materials', 'textures', 'texture.png')
    if not os.path.exists(obj):
        return []
    done = os.path.join(out_dir, f'{tag}3', 'model.sdf')
    if not os.path.exists(done):
        lines = open(obj).read().splitlines()
        v = [ln for ln in lines if ln.startswith('v ')]
        rest = [ln for ln in lines if ln.startswith(('vt ', 'vn '))]
        faces = [ln for ln in lines if ln.startswith('f ')]
        par = list(range(len(v)))

        def find(a):
            while par[a] != a:
                par[a] = par[par[a]]
                a = par[a]
            return a
        fidx = [[int(t.split('/')[0]) - 1 for t in f.split()[1:]] for f in faces]
        for f in fidx:
            r0 = find(f[0])
            for k in f[1:]:
                rk = find(k)
                if rk != r0:
                    par[rk] = r0
        roots = [find(i) for i in range(len(v))]
        comps = sorted(set(roots), key=lambda c: -roots.count(c))
        xyz = [list(map(float, ln.split()[1:4])) for ln in v]
        for n, comp in enumerate(comps[:3], start=1):
            keep = [i for i in range(len(v)) if roots[i] == comp]
            new = {old: k + 1 for k, old in enumerate(keep)}
            cx = sum(xyz[i][0] for i in keep) / len(keep)
            cy = sum(xyz[i][1] for i in keep) / len(keep)
            d = os.path.join(out_dir, f'{tag}{n}')
            os.makedirs(os.path.join(d, 'meshes'), exist_ok=True)
            with open(os.path.join(d, 'meshes', 'doll.mtl'), 'w') as f:
                f.write(f'newmtl material_0\nKd 1 1 1\nmap_Kd {tex}\n')
            with open(os.path.join(d, 'meshes', 'doll.obj'), 'w') as f:
                f.write('mtllib doll.mtl\nusemtl material_0\n')
                for i in keep:
                    x, y, z = xyz[i]
                    f.write(f'v {x - cx:.6f} {y - cy:.6f} {z:.6f}\n')
                f.write('\n'.join(rest) + '\n')
                for fl, fi in zip(faces, fidx):
                    if roots[fi[0]] != comp:
                        continue
                    toks = []
                    for t in fl.split()[1:]:
                        p = t.split('/')
                        p[0] = str(new[int(p[0]) - 1])
                        toks.append('/'.join(p))
                    f.write('f ' + ' '.join(toks) + '\n')
            mesh = (f'<mesh><scale>{scale} {scale} {scale}</scale>'
                    '<uri>meshes/doll.obj</uri></mesh>')
            with open(os.path.join(d, 'model.sdf'), 'w') as f:
                f.write(f'<?xml version="1.0"?><sdf version="1.6"><model name='
                        f'"{tag}{n}"><static>true</static><link name="link">'
                        f'<visual name="visual"><geometry>{mesh}</geometry></visual>'
                        f'<collision name="collision"><geometry>{mesh}</geometry>'
                        f'</collision></link></model></sdf>')
            with open(os.path.join(d, 'model.config'), 'w') as f:
                f.write(f'<?xml version="1.0"?><model><name>{tag}{n}</name>'
                        '<version>1.0</version><sdf version="1.6">model.sdf</sdf>'
                        '</model>')
    return [f'{tag}{n}' for n in (1, 2, 3)]


# Three single dolls in imav2026_scaled (room x -7.7..-2.2, y 9.9..15.4),
# ~3 m apart and all inside the DOWN camera's view from the hold point
# (window axis x -5.83, 1.76 m in -> y 11.66; at 3.85 m it sees about
# +/-2.6 m across and +/-1.9 m fore-aft).
_DOLLS_SCALED = [((-7.30, 13.20, 1.0), 0), ((-3.60, 13.30, -2.0), 1),
                 ((-4.20, 10.60, 2.6), 2)]


def _replace_dolls(sdf, models_dir, out_dir, placements, scale):
    """Comment out the MODERN_DOLL_FAMILY groups, add three single dolls."""
    import re
    sdf = re.sub(r'(<include>\s*<name>baby_doll_\d+</name>.*?</include>)',
                 lambda m: '<!-- ' + m.group(1).replace('--', '- -') + ' -->',
                 sdf, flags=re.S)
    dolls = _doll_models(models_dir, out_dir, scale)
    extra = [f'<include><name>doll_{k + 1}</name><pose>{x} {y} 0.05 0 0 '
             f'{yaw}</pose><uri>model://{dolls[k]}</uri></include>'
             for ((x, y, yaw), k) in placements[:len(dolls)]]
    return sdf.replace('</world>', '\n'.join(extra) + '\n</world>')


def _real_course(sdf, models_dir, out_dir, scale=1.0):
    """v9 -> the measured course: south wall, marker, pad, three dolls."""
    import re
    # Old south wall, window frames and the original doll families: out.
    sdf = re.sub(r'<model name="room_(S_|blue_|red_)[^"]*">.*?</model>', '',
                 sdf, flags=re.S)
    sdf = re.sub(r'(<include>\s*<name>baby_doll_\d+</name>.*?</include>)',
                 lambda m: '<!-- ' + m.group(1).replace('--', '- -') + ' -->',
                 sdf, flags=re.S)
    blue, red, marker, pad = _course(scale)
    for name, (x, y) in (('platform_1', marker), ('takeoff_platform', pad)):
        sdf = re.sub(rf'(<model name="{name}">\s*<static>true</static>\s*<pose>)'
                     r'[-\d.]+ [-\d.]+', rf'\g<1>{x:.3f} {y:.3f}', sdf)
    dolls = _doll_models(models_dir, out_dir)
    extra = [_south_wall(blue, red)]
    for ((x, y, yaw), k) in _DOLLS[:len(dolls)]:
        extra.append(f'<include><name>doll_{k + 1}</name><pose>{x} {y} 0.02 0 0 '
                     f'{yaw}</pose><uri>model://{dolls[k]}</uri></include>')
    return sdf.replace('</world>', '\n'.join(extra) + '\n</world>')


def _patched_world(path, light_scale=1.0, window_scale=1.0):
    """Copy the world to /tmp with any missing PX4 sensor systems added and
    the lighting scaled by light_scale.

    A world that already has every system and is not dimmed is returned as is.
    """
    import re
    with open(path) as f:
        sdf = _dim(f.read(), light_scale)
    add = [f'<plugin filename="{fn}" name="{name}"/>'
           for fn, name in _WORLD_SYSTEMS if fn not in sdf]
    if 'spherical_coordinates' not in sdf:
        add.insert(0, _ORIGIN)
    course = os.path.basename(path).startswith('imav2026_indoor_v9')
    scaled = os.path.basename(path).startswith('imav2026_scaled')
    if not add and abs(light_scale - 1.0) < 1e-6 and not course and not scaled:
        return path
    if scaled:
        sdf = _replace_dolls(sdf, os.path.join(os.path.dirname(path), 'models'),
                             _SIM_MODELS, _DOLLS_SCALED, 10)
    if course:
        sdf = _real_course(sdf, os.path.join(os.path.dirname(path), 'models'),
                           _SIM_MODELS, window_scale)
    # AFTER the world's own systems, as in the worlds where flow works: with
    # the flow system loaded ahead of physics/sensors it never creates its
    # sensor (PX4 subscribes to the flow topic and nothing ever publishes).
    head_end = sdf.find('<model')
    last = None
    for m in re.finditer(r'<plugin\b[^>]*?(/>|>.*?</plugin>)', sdf, re.S):
        if head_end < 0 or m.start() < head_end:
            last = m
    at = last.end() if last else re.search(r'<world[^>]*>', sdf).end()
    if not add:
        at = None
    if at is not None:
        sdf = sdf[:at] + '\n' + '\n'.join(add) + '\n' + sdf[at:]
    out = os.path.join('/tmp', os.path.basename(path).replace('.sdf.world', '_px4.sdf'))
    with open(out, 'w') as f:
        f.write(sdf)
    return out


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    sitl_src = os.path.expanduser(arg('sitl_src'))
    px4_dir = os.path.expanduser(arg('px4_dir'))
    world_name = arg('world')
    world_file = _patched_world(
        os.path.join(sitl_src, 'world', world_name + '.sdf.world'),
        float(arg('light_scale')), float(arg('window_scale')))
    blue, _, marker, pad = _course(float(arg('window_scale')))
    print(f"[sitl] blue opening x {blue[0]:.2f}..{blue[1]:.2f} z {blue[2]:.2f}.."
          f"{blue[3]:.2f}; marker (id 2) {marker[0]:.2f},{marker[1]:.2f}; "
          f"pad (id 0) {pad[0]:.2f},{pad[1]:.2f}", flush=True)
    share = get_package_share_directory('drone_testing')
    urdf = xacro.process_file(
        os.path.join(share, 'sim', 'sim_drone.urdf.xacro')).toxml()
    # COLLISION FOOTPRINT OF THE REAL AIRFRAME (0.26 m), NOT THE x500's. The
    # sim flies PX4's x500 (its dynamics and control allocation), but its
    # rotors sit at +/-0.174 m with 0.14 m props: 0.63 m across, wider than
    # the 0.60 m window, so the props struck the frame mid-traverse. Props no
    # longer collide and the body plate is cut to 0.24 m; the landing gear
    # (0.28 m, 0.227 m below the body) is untouched so it still stands. Pass
    # the node the matching size: gear_below_camera 0.19, drone_height 0.29,
    # drone_width 0.28.
    import re as _re
    urdf = _re.sub(r'<collision[^>]*>(?:(?!</collision>).)*?<box size="0\.2792307692[^"]*"/>'
                   r'.*?</collision>', '', urdf, flags=_re.S)
    urdf = urdf.replace('0.35355339059327373 0.35355339059327373 0.05',
                        '0.24 0.24 0.05')

    # TFmini Plus: 0.1-12 m, ~2 cm noise. The only <max>100.0</max> in the
    # model is the downward lidar's range; the noise goes right after it.
    i = urdf.find('<max>100.0</max>')
    j = urdf.find('</range>', i)
    if i > 0 and j > 0:
        j += len('</range>')
        urdf = (urdf[:i] + '<max>12.0</max>' + urdf[i + len('<max>100.0</max>'):j]
                + '<noise><type>gaussian</type><mean>0</mean>'
                  '<stddev>0.02</stddev></noise>' + urdf[j:])

    plugins = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'src',
                           'modules', 'simulation', 'gz_plugins')
    plugin_dirs = [plugins] + sorted(
        os.path.join(plugins, d) for d in (os.listdir(plugins)
                                           if os.path.isdir(plugins) else [])
        if os.path.isdir(os.path.join(plugins, d)))

    domain = arg('ros_domain')
    env = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', domain),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE',
                               arg('discovery_range')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join([
            os.path.dirname(get_package_share_directory('imav_indoor_2026')),
            os.path.join(sitl_src, 'world'),
            os.path.join(sitl_src, 'world', 'models'), _SIM_MODELS])),
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                  ':'.join(plugin_dirs)),
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch',
            'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v2 {world_file}',
                          'on_exit_shutdown': 'true'}.items())
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': urdf}])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'x500_drone', '-topic', 'robot_description',
                            '-x', arg('x'), '-y', arg('y'), '-z', '0.35',
                            '-Y', arg('yaw')])
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=[
                      '/down_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/down_cam/camera_info@sensor_msgs/msg/CameraInfo'
                      '[gz.msgs.CameraInfo',
                      '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo'
                      '[gz.msgs.CameraInfo',
                      '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                      '/front_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/front_cam/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/front_cam/camera_info@sensor_msgs/msg/CameraInfo'
                      '[gz.msgs.CameraInfo',
                  ],
                  # The RealSense names every consumer already uses.
                  remappings=[
                      ('/front_cam/image', '/camera/camera/color/image_raw'),
                      ('/front_cam/depth_image',
                       '/camera/camera/aligned_depth_to_color/image_raw'),
                      ('/front_cam/camera_info', '/camera/camera/color/camera_info'),
                  ])
    # Flow for x/y, GPS off; arm with no RC/GCS. Height ref is BARO with the
    # rangefinder CONDITIONAL: EKF2 only starts flow fusion with a valid
    # terrain estimate, and with the rangefinder as the height reference
    # (HGT_REF 2) terrain is never estimated -- flow never starts, local
    # position goes invalid at arming and PX4 disarms. This is PX4's
    # recommended flow setup; the rangefinder still gives height above ground.
    # Set twice: as PX4_PARAM_* (read by rcS at boot on PX4 >= 1.14) and
    # again with px4-param once it is up, for builds that ignore the env.
    params = {
        # The DDS client's domain: MUST match ROS_DOMAIN_ID (see ISOLATION).
        'UXRCE_DDS_DOM_ID': int(domain),
        'EKF2_GPS_CTRL': 0, 'EKF2_OF_CTRL': 1, 'EKF2_RNG_CTRL': 1,
        'EKF2_HGT_REF': 1, 'EKF2_MIN_RNG': 0.1, 'COM_ARM_WO_GPS': 1,
        'NAV_RCL_ACT': 0, 'NAV_DLL_ACT': 0, 'COM_RCL_EXCEPT': 4,
        # NO barometer, as on the aircraft: height is the TFmini Plus only,
        # x/y the PMW3901. (EKF2_HGT_REF stays 1 so EKF2 still keeps a
        # terrain estimate from the range, which flow needs to start; with
        # the baro off it falls back to the range for height.)
        'EKF2_BARO_CTRL': 0,
        # No RC in SITL: without this PX4 raises manual_control_signal_lost
        # and drops out of Offboard into Hold.
        'COM_RC_IN_MODE': 4,
        # ONE EKF on one IMU and one compass, as on the aircraft. The sim
        # exposes 3 IMUs and 2 mags, PX4 runs an EKF per pair, and at arming
        # the selector switched to an instance 147 deg out in heading and 2 m
        # out in position -> local position invalid -> auto-disarm. Boot-time
        # only (read at ekf2 start), hence the PX4_PARAM_* route.
        'EKF2_MULTI_IMU': 0, 'SENS_IMU_MODE': 1,
        'EKF2_MULTI_MAG': 0, 'SENS_MAG_MODE': 1,
        # No external vision. The SITL model publishes Gazebo odometry, which
        # PX4's gz_bridge forwards as vehicle_visual_odometry in a different
        # frame (ev_hpos test ratio ~787). Whenever EKF2 tried to start on it
        # it reset position/heading to it -- the 2 m / 147 deg jump at arming
        # and the flapping local_position_invalid. Flow is the only x/y here.
        'EKF2_EV_CTRL': 0,
        # No power module in SITL ("system power unavailable").
        'CBRK_SUPPLY_CHK': 894281,
        # PX4's land detector only accepts touchdown while the descent
        # setpoint is >= 0.9 * MPC_LAND_SPEED. The missions land at 0.10 m/s
        # (slow_land_speed), so at the 0.7 default PX4 never agrees it has
        # landed and refuses the disarm. SET THE SAME ON THE AIRCRAFT.
        'MPC_LAND_SPEED': 0.1,
    }
    env_params = ' '.join(f'PX4_PARAM_{k}={v}' for k, v in params.items())
    set_params = '; '.join(f'bin/px4-param set {k} {v}' for k, v in params.items())
    px4 = ExecuteProcess(
        cmd=['bash', '-c',
             # Kill any PX4 left over from an earlier run first: a second
             # instance publishes the same /fmu/out topics, and the node then
             # sees two vehicles interleaved (flapping failsafes, jumps).
             # exec, so Ctrl-C on the launch reaches PX4 itself.
             f'pkill -x px4; sleep 1; '
             f'cd {px4_dir} && rm -f build/px4_sitl_default/rootfs/*.bson && '
             f'{env_params} PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 '
             f'PX4_GZ_MODEL_NAME=x500_drone PX4_GZ_WORLD={world_name} '
             'exec build/px4_sitl_default/bin/px4 -d'],
        output='screen', sigterm_timeout='5', sigkill_timeout='5')
    px4_params = ExecuteProcess(
        cmd=['bash', '-c',
             f'cd {px4_dir}/build/px4_sitl_default && {set_params}; '
             'echo "SITL PARAMS SET: flow x/y (PMW3901), rangefinder height (TFmini), NO baro, no GPS, no EV, no RC"; '
             f'echo "SITL ISOLATED: ROS_DOMAIN_ID={domain}, localhost only -- export the same in the mission terminal"; '
             # One-shot health report after EKF2 has had time to settle, so a
             # refused arm or a flapping position is explained in this log.
             'sleep 15; echo "===== SITL HEALTH REPORT ====="; '
             'for p in ' + ' '.join(params) + '; do bin/px4-param show $p | grep -E "^ *[x+*]"; done; '
             'bin/px4-commander check; bin/px4-ekf2 status; '
             'bin/px4-listener estimator_status_flags -n 1 | grep -E "cs_(opt_flow|rng_hgt|rng_terrain|baro|mag_hdg|yaw_align|valid_fake|constant)|fs_bad|reject"; '
             'for i in 1 2 3 4 5; do echo "--- sample $i"; '
             'bin/px4-listener estimator_status -n 1 | grep -E "pre_flt_fail|test_ratio|pos_horiz_acc"; '
             'bin/px4-listener estimator_innovation_test_ratios -n 1 | grep -E "flow|heading|gps_h|ev_h|rng|hagl"; '
             'bin/px4-listener vehicle_local_position -n 1 | grep -E " (xy_valid|v_xy_valid|eph|evh|heading_good_for_control):"; '
             'sleep 1; done; '
             'echo "===== END HEALTH REPORT ====="; '
             f'find {plugins} -name libOpticalFlowSystem.so | grep -q . '
             '&& echo "optical flow plugin: found" '
             '|| echo "WARNING: libOpticalFlowSystem.so NOT BUILT -- no optical '
             'flow. sudo apt install libopencv-dev, then make px4_sitl again."'],
        output='screen')
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                           output='screen')
    aruco = Node(package='drone_testing', executable='aruco_pose',
                 name='aruco_pose', output='screen',
                 additional_env={'PYTHONFAULTHANDLER': '1'},
                 parameters=[{'image_topic': '/down_cam/image',
                              'width': 800, 'height': 600,
                              'hfov_deg': 78.0,
                              'marker_id': int(arg('marker_id')),
                              'marker_size': float(arg('marker_size')),
                              'aruco_dict': arg('aruco_dict'),
                              'stream_port': 8080}])
    window = Node(package='drone_testing', executable='window_detect',
                  name='window_detect', output='screen',
                  parameters=[{
                      'image_topic': '/camera/camera/color/image_raw',
                      'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                      'camera_info_topic': '/camera/camera/color/camera_info',
                      'publish_geometry': True,
                      'color': 'blue',
                      'stream_port': 8081,
                  }],
                  condition=IfCondition(arg('window_detect')))
    # The LDS-01 through lidar_loc, exactly as on the aircraft from the scan on:
    # gz_lidar_node (in place of the hls_lfcd driver) -> scan_leveler ->
    # wall_localizer -> pose_kf -> /lidar/odom_kf, which the room scan flies on.
    # Real-arena config (2.5 m room, 0.135 m mount). use_sim_time as in
    # lidar_loc's own sitl_lidar.launch.py.
    # 'arena' = the real 2.5 m room (lidar_arena.yaml); 'sim' = lidar_loc's
    # config for the 2.2x imav2026_scaled world (5.41 m room). Either way the
    # mount is the sim drone's (0.135 m above the body).
    arena = os.path.join(get_package_share_directory('lidar_loc'), 'config',
                         'lidar_rplidar.yaml' if arg('lidar_config') == 'sim'
                         else 'lidar_arena.yaml')
    lidar_on = IfCondition(arg('lidar'))
    lidar = [
        Node(package='lidar_loc', executable='gz_lidar_node', name='gz_lidar_node',
             output='screen', condition=lidar_on,
             parameters=[{'world': world_name, 'model': 'x500_drone',
                          'frame_id': 'lidar_link', 'ros_scan_topic': '/lidar/scan',
                          'gz_scan_topic': '/lidar_2d_v2/scan',
                          'subscribe_scoped_fallback': True, 'publish_clock': True,
                          'publish_ground_truth': True, 'update_rate': 5.5,
                          'use_sim_time': False}]),
        Node(package='lidar_loc', executable='scan_leveler', name='scan_leveler',
             output='screen', condition=lidar_on,
             parameters=[arena, {'use_sim_time': True,
                                 'mount_xyz': [0.0, 0.0, 0.135]}]),
        # Heading from the compass, not the window-gap signature: the gap
        # test picked the wrong wall family here (fix 0.8 m out, frame
        # flipping). seed_yaw_offset = EKF2 heading of arena +X (east) = +90.
        Node(package='lidar_loc', executable='wall_localizer', name='wall_localizer',
             output='screen', condition=lidar_on,
             parameters=[arena, {'use_sim_time': True,
                                 'yaw_source': arg('lidar_yaw_source'),
                                 'seed_yaw_offset': float(arg('lidar_seed_yaw'))}]),
        Node(package='lidar_loc', executable='pose_kf.py', name='pose_kf',
             output='screen', condition=lidar_on,
             parameters=[{'use_sim_time': True}]),
    ]
    return env + lidar + [gazebo, rsp, spawn, bridge, TimerAction(period=6.0, actions=[window]),
                  TimerAction(period=8.0, actions=[px4]),
                  TimerAction(period=20.0, actions=[px4_params]),
                  agent, TimerAction(period=5.0, actions=[aruco])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('discovery_range', default_value='LOCALHOST',
                              description='LOCALHOST (isolated) | SUBNET: lets another '
                                          'machine join domain ros_domain, e.g. the '
                                          'Jetson running doll_detect on its TensorRT '
                                          'engine. Still never domain 0.'),
        DeclareLaunchArgument('ros_domain', default_value='42',
                              description='ROS domain for everything in the sim. '
                                          'Never 0 on a shared network.'),
        DeclareLaunchArgument('sitl_src',
                              default_value='~/ros2_ws/src/imav_indoor_2026_sitl',
                              description='Source checkout of imav_indoor_2026_sitl '
                                          '(the worlds and models are not installed).'),
        DeclareLaunchArgument('px4_dir', default_value='~/PX4-Autopilot'),
        DeclareLaunchArgument('world', default_value='imav2026_indoor_v9'),
        DeclareLaunchArgument('x', default_value='-2.75'),
        DeclareLaunchArgument('y', default_value='-6.47'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('marker_id', default_value='2'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='LDS-01 2D lidar + lidar_loc wall localizer '
                                          '(/lidar/odom_kf) for the room scan.'),
        DeclareLaunchArgument('window_scale', default_value='1.0',
                              description='Blue opening size x this (measured 0.60 m). '
                                          'It is moved to stay inside the 2.5 m wall.'),
        DeclareLaunchArgument('lidar_config', default_value='arena',
                              description="arena (real 2.5 m room) | sim (imav2026_scaled, 5.41 m)"),
        DeclareLaunchArgument('lidar_yaw_source', default_value='imu',
                              description='wall_localizer yaw_source (auto|imu|windows|boot|fixed).'),
        DeclareLaunchArgument('lidar_seed_yaw', default_value='1.5708',
                              description='EKF2 heading of arena +X (east), rad, for yaw_source imu.'),
        DeclareLaunchArgument('light_scale', default_value='0.5',
                              description='Multiplier on every light in the world '
                                          '(1 = as authored). The dark room is dim.'),
        DeclareLaunchArgument('window_detect', default_value='true',
                              description='Start window_detect on the sim front camera '
                                          '(part 2). Browser view on :8081.'),
        DeclareLaunchArgument('marker_size', default_value='0.40',
                              description='Edge of the printed marker in the '
                                          'sim world, not the real 0.80.'),
        DeclareLaunchArgument('aruco_dict', default_value='DICT_5X5_50'),
        OpaqueFunction(function=_setup),
    ])
