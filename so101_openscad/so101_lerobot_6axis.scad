// SO-ARM101 / SO101 LeRobot 6-axis OpenSCAD assembly.
// Source model: TheRobotStudio/SO-ARM100 Simulation/SO101, so101_new_calib.urdf.
// Units: URDF and STL are meters; this view scales everything to millimeters.

$fn = 48;

mm = 1000;

// Pose controls, degrees. These correspond to the six LeRobot joints.
shoulder_pan  = 25;
shoulder_lift = -35;
elbow_flex    = 55;
wrist_flex    = -20;
wrist_roll    = 35;
gripper       = 35;

show_joint_axes = true;
show_labels = true;

printed_yellow = [1.00, 0.82, 0.12, 1.0];
servo_black = [0.08, 0.08, 0.08, 1.0];
axis_red = [0.9, 0.12, 0.10, 1.0];
axis_blue = [0.08, 0.25, 0.95, 1.0];
axis_green = [0.1, 0.62, 0.24, 1.0];

function vmm(v) = [v[0] * mm, v[1] * mm, v[2] * mm];
function deg(v) = [v[0] * 180 / PI, v[1] * 180 / PI, v[2] * 180 / PI];

module pose(xyz, rpy) {
    translate(vmm(xyz))
        rotate(deg(rpy))
            children();
}

module mesh_part(path, rgba) {
    color(rgba)
        scale([mm, mm, mm])
            import(path, convexity = 8);
}

module z_axis_marker(name) {
    if (show_joint_axes) {
        color(axis_blue)
            cylinder(h = 28, r = 2.0, center = true);
        color(axis_red)
            rotate([0, 90, 0])
                cylinder(h = 18, r = 1.0, center = true);
        color(axis_green)
            rotate([90, 0, 0])
                cylinder(h = 18, r = 1.0, center = true);
    }

    if (show_labels) {
        translate([4, -13, 16])
            color([0.05, 0.05, 0.05, 1])
                linear_extrude(height = 0.8)
                    text(name, size = 6, halign = "left", valign = "center");
    }
}

module base_link() {
    pose([-0.00636471, -0.0000994414, -0.0024], [1.5708, -1.67685e-15, 1.5708])
        mesh_part("assets/base_motor_holder_so101_v1.stl", printed_yellow);
    pose([-0.00636471, -8.97657e-09, -0.0024], [1.5708, -2.78073e-29, 1.5708])
        mesh_part("assets/base_so101_v2.stl", printed_yellow);
    pose([0.0263353, -8.97657e-09, 0.0437], [-8.21148e-16, 7.84513e-18, 1.249e-15])
        mesh_part("assets/sts3215_03a_v1.stl", servo_black);
    pose([-0.0309827, -0.000199441, 0.0474], [1.5708, -1.35493e-14, 1.5708])
        mesh_part("assets/waveshare_mounting_plate_so101_v2.stl", printed_yellow);
}

module shoulder_link() {
    pose([-0.0303992, 0.000422241, -0.0417], [1.5708, 1.5708, 0])
        mesh_part("assets/sts3215_03a_v1.stl", servo_black);
    pose([-0.0675992, -0.000177759, 0.0158499], [1.5708, -1.5708, 0])
        mesh_part("assets/motor_holder_so101_base_v1.stl", printed_yellow);
    pose([0.0122008, 0.0000222413, 0.0464], [-1.5708, 2.35221e-33, 0])
        mesh_part("assets/rotation_pitch_so101_v1.stl", printed_yellow);
}

module upper_arm_link() {
    pose([-0.11257, -0.0155, 0.0187], [-3.14159, -5.27356e-16, -1.5708])
        mesh_part("assets/sts3215_03a_v1.stl", servo_black);
    pose([-0.065085, 0.012, 0.0182], [3.14159, 0, -1.30911e-30])
        mesh_part("assets/upper_arm_so101_v1.stl", printed_yellow);
}

module lower_arm_link() {
    pose([-0.0648499, -0.032, 0.0182], [3.14159, 0, 6.67202e-31])
        mesh_part("assets/under_arm_so101_v1.stl", printed_yellow);
    pose([-0.0648499, -0.032, 0.018], [-3.14159, -2.55351e-15, -1.83387e-30])
        mesh_part("assets/motor_holder_so101_wrist_v1.stl", printed_yellow);
    pose([-0.1224, 0.0052, 0.0187], [-3.14159, -7.88861e-31, -3.14159])
        mesh_part("assets/sts3215_03a_v1.stl", servo_black);
}

module wrist_link() {
    pose([8.32667e-17, -0.0424, 0.0306], [1.5708, 1.5708, 0])
        mesh_part("assets/sts3215_03a_no_horn_v1.stl", servo_black);
    pose([0, -0.028, 0.0181], [-1.5708, -1.5708, 0])
        mesh_part("assets/wrist_roll_pitch_so101_v2.stl", printed_yellow);
}

module gripper_link() {
    pose([0.0077, 0.0001, -0.0234], [-1.5708, -5.19179e-17, -1.66533e-16])
        mesh_part("assets/sts3215_03a_v1.stl", servo_black);
    pose([8.32667e-17, -0.000218214, 0.000949706], [-3.14159, -5.55112e-17, 0])
        mesh_part("assets/wrist_roll_follower_so101_v1.stl", printed_yellow);
}

module moving_jaw_link() {
    pose([-5.55112e-17, -5.55112e-17, 0.0189], [9.53145e-17, 6.93889e-18, 1.24077e-24])
        mesh_part("assets/moving_jaw_so101_v1.stl", printed_yellow);
}

module so101() {
    base_link();

    pose([0.0388353, -8.97657e-09, 0.0624], [3.14159, 4.18253e-17, -3.14159]) {
        z_axis_marker("J1 shoulder_pan");
        rotate([0, 0, shoulder_pan]) {
            shoulder_link();

            pose([-0.0303992, -0.0182778, -0.0542], [-1.5708, -1.5708, 0]) {
                z_axis_marker("J2 shoulder_lift");
                rotate([0, 0, shoulder_lift]) {
                    upper_arm_link();

                    pose([-0.11257, -0.028, 1.73763e-16], [-3.63608e-16, 8.74301e-16, 1.5708]) {
                        z_axis_marker("J3 elbow_flex");
                        rotate([0, 0, elbow_flex]) {
                            lower_arm_link();

                            pose([-0.1349, 0.0052, 3.62355e-17], [4.02456e-15, 8.67362e-16, -1.5708]) {
                                z_axis_marker("J4 wrist_flex");
                                rotate([0, 0, wrist_flex]) {
                                    wrist_link();

                                    pose([5.55112e-17, -0.0611, 0.0181], [1.5708, 0.0486795, 3.14159]) {
                                        z_axis_marker("J5 wrist_roll");
                                        rotate([0, 0, wrist_roll]) {
                                            gripper_link();

                                            pose([0.0202, 0.0188, -0.0234], [1.5708, -5.24284e-08, -1.41553e-15]) {
                                                z_axis_marker("J6 gripper");
                                                rotate([0, 0, gripper])
                                                    moving_jaw_link();
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

so101();
