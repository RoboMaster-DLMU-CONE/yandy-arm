use kiss3d::window::Window;
use kiss3d::scene::SceneNode;
use kiss3d::resource::Mesh;
use std::rc::Rc;
use std::cell::RefCell;
use std::path::Path;
use nalgebra as na;
use std::fs::File;
use std::collections::HashMap;

pub struct RobotView {
    pub nodes: Vec<SceneNode>,
    pub end_effector_link: Option<String>,
    pub end_effector_path: Option<std::path::PathBuf>,
}

impl RobotView {
    pub fn new(window: &mut Window, urdf_path: &str) -> Self {
        let robot = urdf_rs::read_file(urdf_path).expect("Failed to read URDF");
        let base_dir = Path::new(urdf_path).parent().unwrap();
        
        let mut view = RobotView { 
            nodes: Vec::new(),
            end_effector_link: None,
            end_effector_path: None,
        };
        
        // Find base link
        let base_link = robot.links.iter().find(|l| l.name == robot.name || l.name == "base_link").or_else(|| robot.links.first());
        
        if let Some(link) = base_link {
             let mut child_map: HashMap<String, Vec<&urdf_rs::Joint>> = HashMap::new();
             for j in &robot.joints {
                 child_map.entry(j.parent.link.clone()).or_default().push(j);
             }
             
             let link_map: HashMap<String, &urdf_rs::Link> = robot.links.iter().map(|l| (l.name.clone(), l)).collect();
             
             // Find end-effector (leaf node with no children)
             for link in &robot.links {
                 if !child_map.contains_key(&link.name) {
                     view.end_effector_link = Some(link.name.clone());
                     // Get mesh path
                     if let Some(visual) = link.visual.first() {
                         if let urdf_rs::Geometry::Mesh { filename, .. } = &visual.geometry {
                             view.end_effector_path = Some(base_dir.join(filename));
                         }
                     }
                     break;
                 }
             }
             
             Self::spawn_link_recursive(window, &link_map[&link.name], &link_map, &child_map, base_dir, na::Isometry3::identity(), &mut view.nodes);
        }
        
        view
    }
    
    fn spawn_link_recursive(
        window: &mut Window, 
        link: &urdf_rs::Link,
        link_map: &HashMap<String, &urdf_rs::Link>,
        child_map: &HashMap<String, Vec<&urdf_rs::Joint>>,
        base_dir: &Path,
        parent_transform: na::Isometry3<f32>,
        nodes: &mut Vec<SceneNode>
    ) {
        // Visuals
        for visual in &link.visual {
            let geom = &visual.geometry;
            match geom {
                urdf_rs::Geometry::Mesh { filename, scale } => {
                    let mesh_path = base_dir.join(filename);
                    if let Some(mesh) = load_stl(&mesh_path) {
                        let mut node = window.add_mesh(mesh, na::Vector3::new(1.0, 1.0, 1.0));
                        
                        // Visual origin
                        let rpy = &visual.origin.rpy;
                        let xyz = &visual.origin.xyz;
                        
                        let rotation = na::UnitQuaternion::from_euler_angles(rpy[0] as f32, rpy[1] as f32, rpy[2] as f32);
                        let translation = na::Translation3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);
                        let visual_transform = na::Isometry3::from_parts(translation, rotation);
                        
                        // Combine: Parent * Visual
                        let final_transform = parent_transform * visual_transform;
                        
                        node.set_local_transformation(final_transform);
                        
                        // Apply scale
                        if let Some(s) = scale {
                             node.set_local_scale(s[0] as f32, s[1] as f32, s[2] as f32);
                        }
                        
                        // Color
                        if let Some(mat) = &visual.material {
                            if let Some(color) = &mat.color {
                                node.set_color(color.rgba[0] as f32, color.rgba[1] as f32, color.rgba[2] as f32);
                            }
                        } else {
                            node.set_color(0.5, 0.5, 0.5);
                        }
                        
                        nodes.push(node);
                    } else {
                        eprintln!("Failed to load mesh: {:?}", mesh_path);
                    }
                },
                _ => {}
            }
        }
        
        // Children
        if let Some(joints) = child_map.get(&link.name) {
            for joint in joints {
                if let Some(child_link) = link_map.get(&joint.child.link) {
                    let xyz = &joint.origin.xyz;
                    let rpy = &joint.origin.rpy;
                    
                    let rotation = na::UnitQuaternion::from_euler_angles(rpy[0] as f32, rpy[1] as f32, rpy[2] as f32);
                    let translation = na::Translation3::new(xyz[0] as f32, xyz[1] as f32, xyz[2] as f32);
                    let joint_transform = na::Isometry3::from_parts(translation, rotation);
                    
                    let next_transform = parent_transform * joint_transform;
                    Self::spawn_link_recursive(window, child_link, link_map, child_map, base_dir, next_transform, nodes);
                }
            }
        }
    }
}

fn load_stl(path: &Path) -> Option<Rc<RefCell<Mesh>>> {
    let mut file = File::open(path).ok()?;
    let stl = stl_io::read_stl(&mut file).ok()?;
    
    let mut coords = Vec::new();
    let mut indices = Vec::new();
    
    // Naive conversion: Unweld vertices
    for (i, face) in stl.faces.iter().enumerate() {
        for &v_idx in &face.vertices {
             let v = stl.vertices[v_idx];
             coords.push(na::Point3::new(v[0], v[1], v[2]));
        }
        let base = i as u16 * 3;
        indices.push(na::Point3::new(base, base + 1, base + 2));
    }
    
    Some(Rc::new(RefCell::new(Mesh::new(coords, indices, None, None, false))))
}
