//! Decoding of ROS 2 messages carried over Zenoh by rmw_zenoh into JSON text.
//!
//! The message type is read from the key (`<domain>/<topic>/<pkg>::msg::dds_::<Name>_/<hash>`).
//! Its `.msg` definition comes from a local folder, the download cache or GitHub
//! (see `lookup`). Definitions load on a background thread: until one is ready,
//! `decode` returns None and the caller shows the payload as before.

mod cdr;
mod lookup;
mod parse;

use std::collections::{HashMap, HashSet};
use std::path::Path;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering::Relaxed};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, OnceLock, RwLock};
use std::time::{Duration, Instant};

use tracing::{debug, info, warn};

use crate::types::MAX_UI_DISPLAY_SIZE;
use parse::MsgDef;

/// The last segment of a Humble rmw_zenoh key, where a type hash would be.
const NO_HASH_SEGMENT: &str = "TypeHashNotSupported";
/// How long a definition that failed to load is left alone before the next try.
const RETRY_AFTER: Duration = Duration::from_secs(30);
/// Most definitions loaded for one request, a guard against runaway definitions.
const MAX_LOADED_PER_REQUEST: usize = 500;

#[derive(Debug, Clone, PartialEq)]
pub struct Settings {
    /// Folder scanned for `<package>/msg/*.msg`; empty for none.
    pub msg_dir: String,
    /// Fetch standard definitions from GitHub when they are not found locally.
    pub online: bool,
    /// ROS distro whose branch the GitHub lookup reads.
    pub distro: String,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            msg_dir: String::new(),
            online: true,
            distro: "humble".into(),
        }
    }
}

enum Entry {
    Pending,
    /// Loaded, and from where: "local", "cache" or "github".
    Ready(Arc<MsgDef>, &'static str),
    Failed(Instant, String),
}

enum Job {
    Rescan,
    Load(String, u64),
}

struct Inner {
    settings: RwLock<Settings>,
    /// Counts settings changes; a load that started under an older one is dropped.
    generation: AtomicU64,
    defs: RwLock<HashMap<String, Entry>>,
    /// Definitions found in the local folder by the last scan.
    local_found: AtomicUsize,
    /// Messages with at least one payload decoded, for the key label.
    decoded: RwLock<HashSet<String>>,
}

pub struct Registry {
    inner: Arc<Inner>,
    jobs: Sender<Job>,
}

impl Registry {
    pub fn new() -> Self {
        let inner = Arc::new(Inner {
            settings: RwLock::new(Settings::default()),
            generation: AtomicU64::new(0),
            defs: RwLock::new(HashMap::new()),
            local_found: AtomicUsize::new(0),
            decoded: RwLock::new(HashSet::new()),
        });
        let (jobs, rx) = channel();
        let worker = inner.clone();
        std::thread::Builder::new()
            .name("ros-msg-lookup".into())
            .spawn(move || worker.run(rx))
            .expect("spawn ros-msg-lookup thread");
        Self { inner, jobs }
    }

    /// Use new settings: forget what was loaded and rescan the local folder.
    pub fn apply_settings(&self, settings: Settings) {
        *self.inner.settings.write().unwrap() = settings;
        self.inner.generation.fetch_add(1, Relaxed);
        self.inner.defs.write().unwrap().clear();
        self.inner.decoded.write().unwrap().clear();
        let _ = self.jobs.send(Job::Rescan);
    }

    /// The payload of a sample on `key` as JSON text, or None when the key is not
    /// a ROS 2 message, its definition is not loaded yet, or the bytes do not fit it.
    pub fn decode(&self, key: &str, payload: &[u8]) -> Option<String> {
        if payload.len() < 4 {
            return None;
        }
        let name = type_from_key(key)?;
        self.get(&name)?;
        let value = cdr::decode(payload, &name, &|n| self.get(n))
            .inspect_err(|e| debug!("ROS decode of {key} failed: {e}"))
            .ok()?;
        let text = serde_json::to_string(&value).ok()?;
        if !self.inner.decoded.read().unwrap().contains(&name) {
            self.inner.decoded.write().unwrap().insert(name);
        }
        (text.len() <= MAX_UI_DISPLAY_SIZE).then_some(text)
    }

    /// What to show in place of the `TypeHashNotSupported` segment of `key`; None
    /// when the key does not end in it. Humble sends no type hash, so the label says
    /// instead whether Explorer decodes this type, and from where its definition came.
    pub fn hash_label(&self, key: &str) -> Option<&'static str> {
        key.strip_suffix(NO_HASH_SEGMENT)?.ends_with('/').then_some(())?;
        let name = type_from_key(key)?;
        let source = match self.inner.defs.read().unwrap().get(&name) {
            Some(Entry::Ready(_, source)) if self.inner.decoded.read().unwrap().contains(&name) => {
                Some(*source)
            }
            _ => None,
        };
        Some(match source {
            Some("local") => "decoded · local folder",
            Some("cache") => "decoded · cache",
            Some(_) => "decoded · GitHub",
            None => "no type hash (Humble)",
        })
    }

    /// One line for the settings panel: what is loaded, and what is missing and why.
    pub fn status(&self) -> String {
        let defs = self.inner.defs.read().unwrap();
        let mut counts = [0usize; 3];
        let mut missing = Vec::new();
        for (name, entry) in defs.iter() {
            match entry {
                Entry::Ready(_, "local") => counts[0] += 1,
                Entry::Ready(_, "cache") => counts[1] += 1,
                Entry::Ready(..) => counts[2] += 1,
                Entry::Failed(_, e) => missing.push(format!("{name} ({e})")),
                Entry::Pending => {}
            }
        }
        missing.sort();
        let mut text = format!(
            "ROS types loaded: {} (local {}, cache {}, github {}) · local folder: {} found",
            counts.iter().sum::<usize>(),
            counts[0],
            counts[1],
            counts[2],
            self.inner.local_found.load(Relaxed),
        );
        if !missing.is_empty() {
            text.push_str(&format!(" · missing: {}", missing.join(", ")));
        }
        text
    }

    /// The definition of `name` if loaded; otherwise ask the worker for it.
    fn get(&self, name: &str) -> Option<Arc<MsgDef>> {
        match self.inner.defs.read().unwrap().get(name) {
            Some(Entry::Ready(def, _)) => return Some(def.clone()),
            Some(Entry::Pending) => return None,
            Some(Entry::Failed(at, _)) if at.elapsed() < RETRY_AFTER => return None,
            _ => {}
        }
        let mut defs = self.inner.defs.write().unwrap();
        match defs.get(name) {
            Some(Entry::Ready(def, _)) => return Some(def.clone()),
            Some(Entry::Pending) => return None,
            Some(Entry::Failed(at, _)) if at.elapsed() < RETRY_AFTER => return None,
            _ => {}
        }
        defs.insert(name.to_string(), Entry::Pending);
        let generation = self.inner.generation.load(Relaxed);
        let _ = self.jobs.send(Job::Load(name.to_string(), generation));
        None
    }
}

impl Inner {
    fn run(&self, jobs: Receiver<Job>) {
        let mut index = HashMap::new();
        for job in jobs {
            match job {
                Job::Rescan => {
                    let dir = self.settings.read().unwrap().msg_dir.trim().to_string();
                    index = if dir.is_empty() {
                        HashMap::new()
                    } else {
                        lookup::scan_dir(Path::new(&dir))
                    };
                    self.local_found.store(index.len(), Relaxed);
                    info!("ROS msg folder '{dir}': {} definitions found", index.len());
                }
                Job::Load(name, generation) => self.load(&index, &name, generation),
            }
        }
    }

    /// Load `root` and every message it contains.
    fn load(&self, index: &HashMap<String, std::path::PathBuf>, root: &str, generation: u64) {
        let settings = self.settings.read().unwrap().clone();
        let mut todo = vec![root.to_string()];
        let mut done = HashSet::new();
        while let Some(name) = todo.pop() {
            if !done.insert(name.clone()) || done.len() > MAX_LOADED_PER_REQUEST {
                continue;
            }
            let result = read_def(&settings, index, &name).and_then(|(text, source)| {
                let pkg = name.split('/').next().unwrap_or_default();
                parse::parse(pkg, &text).map(|def| (def, source))
            });
            let mut defs = self.defs.write().unwrap();
            if self.generation.load(Relaxed) != generation {
                return; // settings changed meanwhile
            }
            match result {
                Ok((def, source)) => {
                    for n in def.nested() {
                        let wanted = match defs.get(n) {
                            None => true,
                            Some(Entry::Failed(at, _)) => at.elapsed() >= RETRY_AFTER,
                            Some(_) => false,
                        };
                        if wanted {
                            todo.push(n.to_string());
                        }
                    }
                    info!("ROS type {name} loaded ({source})");
                    defs.insert(name, Entry::Ready(Arc::new(def), source));
                }
                Err(e) => {
                    warn!("ROS type {name}: {e}");
                    defs.insert(name, Entry::Failed(Instant::now(), e));
                }
            }
        }
    }
}

/// The text of `package/Name` and where it came from: the local folder, then the
/// download cache, then GitHub.
fn read_def(
    settings: &Settings,
    index: &HashMap<String, std::path::PathBuf>,
    name: &str,
) -> Result<(String, &'static str), String> {
    let (pkg, short) = name.split_once('/').ok_or("bad message name")?;
    if let Some(path) = index.get(name) {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        return Ok((text, "local"));
    }
    let distro = settings.distro.trim();
    if !lookup::is_valid_distro(distro) {
        return Err(format!("'{distro}' is not a valid ROS distro name"));
    }
    let cached = lookup::cache_path(&lookup::cache_dir(), distro, pkg, short);
    if let Ok(text) = std::fs::read_to_string(&cached) {
        return Ok((text, "cache"));
    }
    if !settings.online {
        return Err("not in the local folder or cache, and online lookup is off".into());
    }
    let url = lookup::github_url(pkg, short, distro)
        .ok_or_else(|| format!("{pkg} is not a standard package; put it in the local folder"))?;
    let text = lookup::fetch(&url).map_err(|e| format!("{url}: {e}"))?;
    parse::parse(pkg, &text).map_err(|e| format!("{url}: {e}"))?;
    // Not worth failing the load when the cache cannot be written.
    if std::fs::create_dir_all(cached.parent().unwrap_or(Path::new("."))).is_ok() {
        let _ = std::fs::write(&cached, &text);
    }
    Ok((text, "github"))
}

/// `package/Name` from the type segment of a rmw_zenoh key, if it has one.
fn type_from_key(key: &str) -> Option<String> {
    let segment = key.split('/').find(|s| s.contains("::msg::dds_::"))?;
    let (pkg, rest) = segment.split_once("::msg::dds_::")?;
    let name = rest.strip_suffix('_')?;
    (parse::is_ident(pkg) && parse::is_ident(name)).then(|| format!("{pkg}/{name}"))
}

fn global() -> &'static Registry {
    static REGISTRY: OnceLock<Registry> = OnceLock::new();
    REGISTRY.get_or_init(Registry::new)
}

/// See [`Registry::decode`].
pub fn decode(key: &str, payload: &[u8]) -> Option<String> {
    global().decode(key, payload)
}

/// See [`Registry::apply_settings`].
pub fn apply_settings(settings: Settings) {
    global().apply_settings(settings);
}

/// See [`Registry::hash_label`].
pub fn hash_label(key: &str) -> Option<&'static str> {
    global().hash_label(key)
}

/// `key` for display: its `TypeHashNotSupported` segment, if any, becomes its label.
pub fn display_key(key: &str) -> std::borrow::Cow<'_, str> {
    match hash_label(key) {
        Some(label) => format!("{}{label}", &key[..key.len() - NO_HASH_SEGMENT.len()]).into(),
        None => key.into(),
    }
}

/// See [`Registry::status`].
pub fn status() -> String {
    global().status()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn type_comes_from_the_key() {
        assert_eq!(
            type_from_key("0/clock_raw/rosgraph_msgs::msg::dds_::Clock_/TypeHashNotSupported"),
            Some("rosgraph_msgs/Clock".into())
        );
        assert_eq!(
            type_from_key("0/a/b/tf2_msgs::msg::dds_::TFMessage_/x"),
            Some("tf2_msgs/TFMessage".into())
        );
        assert_eq!(type_from_key("demo/test"), None);
        assert_eq!(type_from_key("0/x/pkg::srv::dds_::Do_Request_/x"), None);
        assert_eq!(type_from_key("0/x/..%2fevil::msg::dds_::A_/x"), None);
    }

    fn wait_for(r: &Registry, key: &str, payload: &[u8]) -> Option<String> {
        for _ in 0..100 {
            if let Some(text) = r.decode(key, payload) {
                return Some(text);
            }
            std::thread::sleep(Duration::from_millis(30));
        }
        None
    }

    #[test]
    fn decodes_from_a_local_folder_and_reports_missing_types() {
        let root = std::env::temp_dir().join("zenoh_explorer_test_registry");
        let msg = root.join("zx_test_msgs/msg");
        std::fs::create_dir_all(&msg).unwrap();
        std::fs::write(msg.join("Outer.msg"), "Inner inner\nstring label\n").unwrap();
        std::fs::write(msg.join("Inner.msg"), "uint8 id\nfloat32 gain\n").unwrap();

        let r = Registry::new();
        r.apply_settings(Settings {
            msg_dir: root.display().to_string(),
            online: false,
            distro: "humble".into(),
        });
        let mut payload = vec![0, 1, 0, 0, 3, 0, 0, 0];
        payload.extend(0.5f32.to_le_bytes()); // gain at offset 4
        payload.extend(2u32.to_le_bytes());
        payload.extend(*b"a\0");
        let key = "0/t/zx_test_msgs::msg::dds_::Outer_/TypeHashNotSupported";
        assert_eq!(
            wait_for(&r, key, &payload).unwrap(),
            r#"{"inner":{"id":3,"gain":0.5},"label":"a"}"#
        );
        assert!(r.status().contains("local 2"), "{}", r.status());
        assert_eq!(r.hash_label(key), Some("decoded · local folder"));
        assert_eq!(r.hash_label("0/t/Outer/other"), None);

        // Not in the folder, and the lookup is off: stays undecoded and is reported.
        let key = "0/t/zx_test_msgs::msg::dds_::Absent_/x";
        assert_eq!(wait_for(&r, key, &payload), None);
        assert!(r.status().contains("zx_test_msgs/Absent"), "{}", r.status());
        assert_eq!(
            r.hash_label("0/t/zx_test_msgs::msg::dds_::Absent_/TypeHashNotSupported"),
            Some("no type hash (Humble)")
        );
    }
}
