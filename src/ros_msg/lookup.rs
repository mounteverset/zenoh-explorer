//! Where `.msg` text comes from: a local folder, the download cache, GitHub.

use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::Duration;

use super::parse::is_ident;

/// Largest definition file accepted from the network.
const MAX_DEF_BYTES: u64 = 256 * 1024;
/// Most files looked at when scanning a local folder.
const MAX_SCANNED: usize = 200_000;
const MAX_SCAN_DEPTH: usize = 12;

/// Repositories holding the standard packages, whose `msg/` is at `<package>/msg`.
const REPOS: &[(&str, &[&str])] = &[
    (
        "ros2/common_interfaces",
        &[
            "actionlib_msgs",
            "diagnostic_msgs",
            "geometry_msgs",
            "nav_msgs",
            "sensor_msgs",
            "shape_msgs",
            "std_msgs",
            "stereo_msgs",
            "trajectory_msgs",
            "visualization_msgs",
        ],
    ),
    (
        "ros2/rcl_interfaces",
        &[
            "action_msgs",
            "builtin_interfaces",
            "lifecycle_msgs",
            "rcl_interfaces",
            "rosgraph_msgs",
            "statistics_msgs",
        ],
    ),
    ("ros2/geometry2", &["tf2_msgs"]),
    ("ros-planning/navigation2", &["nav2_msgs"]),
    ("ros-controls/control_msgs", &["control_msgs"]),
    ("ANYbotics/grid_map", &["grid_map_msgs"]),
];

/// Single-package repositories, whose `msg/` is at the repository root.
const ROOT_REPOS: &[(&str, &str)] = &[
    ("unique_identifier_msgs", "ros2/unique_identifier_msgs"),
    ("example_interfaces", "ros2/example_interfaces"),
];

/// A distro name is used in a URL and a file path, so only plain names pass.
pub fn is_valid_distro(distro: &str) -> bool {
    is_ident(&distro.replace('-', "_"))
}

/// The raw GitHub URL of a standard message, or None for an unknown package.
/// `pkg`, `name` and `distro` must already be valid.
pub fn github_url(pkg: &str, name: &str, distro: &str) -> Option<String> {
    let path = match ROOT_REPOS.iter().find(|(p, _)| *p == pkg) {
        Some((_, repo)) => format!("{repo}/{distro}/msg/{name}.msg"),
        None => {
            let (repo, _) = REPOS.iter().find(|(_, pkgs)| pkgs.contains(&pkg))?;
            format!("{repo}/{distro}/{pkg}/msg/{name}.msg")
        }
    };
    Some(format!("https://raw.githubusercontent.com/{path}"))
}

pub fn fetch(url: &str) -> Result<String, String> {
    let response = ureq::AgentBuilder::new()
        .timeout(Duration::from_secs(10))
        .redirects(0)
        .build()
        .get(url)
        .call()
        .map_err(|e| e.to_string())?;
    let mut text = String::new();
    response
        .into_reader()
        .take(MAX_DEF_BYTES)
        .read_to_string(&mut text)
        .map_err(|e| e.to_string())?;
    Ok(text)
}

/// Where downloaded definitions are kept between runs.
pub fn cache_dir() -> PathBuf {
    let var = |k| std::env::var_os(k).map(PathBuf::from);
    var("XDG_CACHE_HOME")
        .or_else(|| var("HOME").map(|h| h.join(".cache")))
        .or_else(|| var("LOCALAPPDATA"))
        .unwrap_or_else(std::env::temp_dir)
        .join("zenoh-explorer")
        .join("ros_msgs")
}

pub fn cache_path(root: &Path, distro: &str, pkg: &str, name: &str) -> PathBuf {
    root.join(distro).join(pkg).join(format!("{name}.msg"))
}

/// Find every `<package>/msg/<Name>.msg` below `root`, as `package/Name`.
/// The first file found wins when a message appears twice (source and install trees).
pub fn scan_dir(root: &Path) -> HashMap<String, PathBuf> {
    let mut found = HashMap::new();
    let mut seen = 0;
    let mut stack = vec![(root.to_path_buf(), 0)];
    while let Some((dir, depth)) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            seen += 1;
            if seen > MAX_SCANNED {
                return found;
            }
            let path = entry.path();
            let hidden = entry.file_name().to_string_lossy().starts_with('.');
            // Symlinks are skipped, so a link back up the tree cannot loop.
            let Ok(kind) = entry.file_type() else { continue };
            if hidden || kind.is_symlink() {
                continue;
            }
            if kind.is_dir() {
                if depth < MAX_SCAN_DEPTH {
                    stack.push((path, depth + 1));
                }
            } else if path.extension().is_some_and(|e| e == "msg") {
                let parent = path.parent();
                let pkg = parent
                    .filter(|p| p.file_name().is_some_and(|n| n == "msg"))
                    .and_then(|p| p.parent())
                    .and_then(|p| p.file_name())
                    .and_then(|n| n.to_str());
                let name = path.file_stem().and_then(|n| n.to_str());
                if let (Some(pkg), Some(name)) = (pkg, name) {
                    if is_ident(pkg) && is_ident(name) {
                        found.entry(format!("{pkg}/{name}")).or_insert(path);
                    }
                }
            }
        }
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn urls_follow_the_repository_layout() {
        assert_eq!(
            github_url("geometry_msgs", "Twist", "humble").unwrap(),
            "https://raw.githubusercontent.com/ros2/common_interfaces/humble/geometry_msgs/msg/Twist.msg"
        );
        assert_eq!(
            github_url("unique_identifier_msgs", "UUID", "humble").unwrap(),
            "https://raw.githubusercontent.com/ros2/unique_identifier_msgs/humble/msg/UUID.msg"
        );
        assert!(github_url("my_robot_msgs", "Thing", "humble").is_none());
    }

    #[test]
    fn distro_names_are_plain() {
        assert!(is_valid_distro("humble"));
        assert!(!is_valid_distro("../x"));
        assert!(!is_valid_distro(""));
    }

    #[test]
    fn scan_finds_packages_by_folder_name() {
        let root = std::env::temp_dir().join("zenoh_explorer_test_scan");
        let msg = root.join("src/robot_msgs/msg");
        std::fs::create_dir_all(&msg).unwrap();
        std::fs::write(msg.join("Pose2.msg"), "float64 x\n").unwrap();
        std::fs::write(msg.join("notes.txt"), "x").unwrap();
        std::fs::create_dir_all(root.join("src/robot_msgs/srv")).unwrap();
        std::fs::write(root.join("src/robot_msgs/srv/Thing.msg"), "x").unwrap();
        let found = scan_dir(&root);
        assert_eq!(found.len(), 1);
        assert!(found["robot_msgs/Pose2"].ends_with("src/robot_msgs/msg/Pose2.msg"));
    }

    /// Needs the network: fetches two small files from GitHub.
    #[test]
    #[ignore]
    fn fetches_standard_definitions() {
        for (pkg, name) in [
            ("geometry_msgs", "Twist"),
            ("rosgraph_msgs", "Clock"),
            ("tf2_msgs", "TFMessage"),
            ("nav2_msgs", "SpeedLimit"),
            ("unique_identifier_msgs", "UUID"),
            ("control_msgs", "DynamicJointState"),
            ("control_msgs", "SteeringControllerStatus"),
            ("grid_map_msgs", "GridMap"),
        ] {
            let url = github_url(pkg, name, "humble").unwrap();
            let text = fetch(&url).unwrap_or_else(|e| panic!("{url}: {e}"));
            assert!(super::super::parse::parse(pkg, &text).is_ok(), "{url}");
        }
    }
}
