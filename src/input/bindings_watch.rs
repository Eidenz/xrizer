//! Hot reload for personal bindings.
//!
//! A game's personal bindings (`xrizer/<controller>.json` in its working
//! directory, or `$XRIZER_CUSTOM_BINDINGS_DIR`) are read when it loads its
//! action manifest, and OpenXR fixes an action set's bindings once it's
//! attached to a session. So a change on disk takes a new session: a thread
//! watches the folder (inotify, or a check every couple of seconds where that's
//! unavailable), and when one of those files really changed (written, created
//! or removed; the same bytes saved again don't count) it raises a flag. Between
//! two frames the compositor sees it and restarts the session, which loads the
//! manifest, and so the bindings, again.

use log::{info, warn};
use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::os::unix::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

/// A save is a burst of events (several files, temp file + rename): wait this
/// long after the last one before looking.
const SETTLE: Duration = Duration::from_millis(100);
/// Without inotify, look this often.
const POLL: Duration = Duration::from_secs(2);

/// On the bindings folder: its files being written, replaced or removed, and
/// the folder itself going away.
const FILE_EVENTS: u32 = libc::IN_CLOSE_WRITE
    | libc::IN_CREATE
    | libc::IN_MOVED_TO
    | libc::IN_MOVED_FROM
    | libc::IN_DELETE
    | libc::IN_DELETE_SELF
    | libc::IN_MOVE_SELF
    | libc::IN_ONLYDIR;
/// On its parent: the folder appearing or disappearing.
const DIR_EVENTS: u32 =
    libc::IN_CREATE | libc::IN_MOVED_TO | libc::IN_MOVED_FROM | libc::IN_DELETE | libc::IN_ONLYDIR;

pub(super) struct BindingsWatch {
    changed: Arc<AtomicBool>,
    /// An eventfd the thread also waits on: written on drop to stop it.
    stop: Arc<OwnedFd>,
}

impl BindingsWatch {
    /// Watch `files` (names) in `dir`, which may not exist yet.
    pub fn start(dir: PathBuf, files: Vec<String>) -> Option<Self> {
        // Miri can't make the calls this needs (inotify, poll).
        if cfg!(miri) {
            return None;
        }
        // SAFETY: plain syscall; the descriptor is owned right away.
        let raw = unsafe { libc::eventfd(0, libc::EFD_CLOEXEC | libc::EFD_NONBLOCK) };
        if raw < 0 {
            warn!(
                "Not watching personal bindings for changes: {}",
                io::Error::last_os_error()
            );
            return None;
        }
        let stop = Arc::new(unsafe { OwnedFd::from_raw_fd(raw) });
        let changed = Arc::new(AtomicBool::new(false));
        let (thread_changed, thread_stop) = (changed.clone(), stop.clone());
        std::thread::Builder::new()
            .name("xrizer-bindings".into())
            .spawn(move || run(dir, files, &thread_changed, thread_stop.as_raw_fd()))
            .inspect_err(|e| warn!("Not watching personal bindings for changes: {e}"))
            .ok()?;
        Some(Self { changed, stop })
    }

    /// Whether the bindings changed since the last [`Self::take`].
    pub fn pending(&self) -> bool {
        self.changed.load(Ordering::Acquire)
    }

    pub fn take(&self) -> bool {
        self.changed.swap(false, Ordering::AcqRel)
    }
}

impl Drop for BindingsWatch {
    fn drop(&mut self) {
        let one: u64 = 1;
        // SAFETY: an 8-byte write from a live u64 to our eventfd.
        unsafe { libc::write(self.stop.as_raw_fd(), (&raw const one).cast(), 8) };
    }
}

enum Woke {
    /// Something may have changed: compare the files.
    Check,
    Stop,
}

fn run(dir: PathBuf, files: Vec<String>, changed: &AtomicBool, stop: RawFd) {
    let read = || -> Vec<Option<Vec<u8>>> {
        files
            .iter()
            .map(|f| std::fs::read(dir.join(f)).ok())
            .collect()
    };
    let mut seen = read();
    let mut notify = match Inotify::new(&dir) {
        Ok(n) => {
            info!("Watching {} for binding changes", dir.display());
            Some(n)
        }
        Err(e) => {
            warn!(
                "Can't watch {} ({e}), checking it every {}s instead",
                dir.display(),
                POLL.as_secs()
            );
            None
        }
    };
    loop {
        let woke = match notify.as_mut().map(|n| n.wait(stop, &files)) {
            Some(Ok(w)) => w,
            Some(Err(e)) => {
                warn!(
                    "Lost the watch on {} ({e}), checking it every {}s instead",
                    dir.display(),
                    POLL.as_secs()
                );
                notify = None;
                Woke::Check
            }
            None => match wait_fds(&mut [pollfd(stop)], Some(POLL)) {
                Ok(0) => Woke::Check,
                _ => Woke::Stop,
            },
        };
        if let Woke::Stop = woke {
            return;
        }
        let now = read();
        if now != seen {
            seen = now;
            info!("Personal bindings changed in {}", dir.display());
            changed.store(true, Ordering::Release);
        }
    }
}

fn pollfd(fd: RawFd) -> libc::pollfd {
    libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    }
}

/// Wait for any of `fds` to be readable, up to `timeout` (`None`: forever).
/// Returns how many are.
fn wait_fds(fds: &mut [libc::pollfd], timeout: Option<Duration>) -> io::Result<usize> {
    let ms = timeout.map_or(-1, |t| t.as_millis().try_into().unwrap_or(i32::MAX));
    loop {
        // SAFETY: `fds` is a valid slice of pollfds for the whole call.
        let n = unsafe { libc::poll(fds.as_mut_ptr(), fds.len() as libc::nfds_t, ms) };
        if n >= 0 {
            return Ok(n as usize);
        }
        let e = io::Error::last_os_error();
        if e.kind() != io::ErrorKind::Interrupted {
            return Err(e);
        }
    }
}

struct Inotify {
    fd: OwnedFd,
    dir: PathBuf,
    dir_name: Vec<u8>,
    /// The folder's parent, to see the folder appear (and come back).
    parent_wd: Option<i32>,
    dir_wd: Option<i32>,
    buf: Vec<u8>,
}

impl Inotify {
    fn new(dir: &Path) -> io::Result<Self> {
        // SAFETY: plain syscall; the descriptor is owned right away.
        let raw = unsafe { libc::inotify_init1(libc::IN_CLOEXEC | libc::IN_NONBLOCK) };
        if raw < 0 {
            return Err(io::Error::last_os_error());
        }
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        let parent_wd = dir
            .parent()
            .and_then(|p| add_watch(&fd, p, DIR_EVENTS).ok());
        let dir_wd = add_watch(&fd, dir, FILE_EVENTS).ok();
        if parent_wd.is_none() && dir_wd.is_none() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                "neither the folder nor its parent exists",
            ));
        }
        Ok(Self {
            fd,
            dir: dir.to_path_buf(),
            dir_name: dir
                .file_name()
                .map(|n| n.as_bytes().to_vec())
                .unwrap_or_default(),
            parent_wd,
            dir_wd,
            // Room for a few events with the longest names.
            buf: vec![0; 4096],
        })
    }

    /// Sleep until something happened to one of `files` (or to the folder),
    /// then until the burst settles. `Stop` when `stop` is written.
    fn wait(&mut self, stop: RawFd, files: &[String]) -> io::Result<Woke> {
        let mut due = false;
        loop {
            let mut fds = [pollfd(stop), pollfd(self.fd.as_raw_fd())];
            if wait_fds(&mut fds, due.then_some(SETTLE))? == 0 {
                return Ok(Woke::Check);
            }
            if fds[0].revents != 0 {
                return Ok(Woke::Stop);
            }
            if fds[1].revents != 0 {
                due |= self.read_events(files)?;
            }
        }
    }

    /// Whether the events waiting concern the bindings.
    fn read_events(&mut self, files: &[String]) -> io::Result<bool> {
        // SAFETY: reading into our own buffer, at most its length.
        let n = unsafe {
            libc::read(
                self.fd.as_raw_fd(),
                self.buf.as_mut_ptr().cast(),
                self.buf.len(),
            )
        };
        if n < 0 {
            let e = io::Error::last_os_error();
            return match e.kind() {
                io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock => Ok(false),
                _ => Err(e),
            };
        }
        let (n, head) = (n as usize, size_of::<libc::inotify_event>());
        let mut relevant = false;
        let mut off = 0;
        while off + head <= n {
            // SAFETY: the kernel writes whole events; the header may be unaligned.
            let ev: libc::inotify_event =
                unsafe { std::ptr::read_unaligned(self.buf.as_ptr().add(off).cast()) };
            let end = (off + head + ev.len as usize).min(n);
            let name = self.buf[off + head..end]
                .split(|&b| b == 0)
                .next()
                .unwrap_or(&[]);
            off = end;
            if Some(ev.wd) == self.dir_wd {
                if ev.mask & (libc::IN_DELETE_SELF | libc::IN_MOVE_SELF | libc::IN_IGNORED) != 0 {
                    // The folder went away (a moved one is still watched: drop that).
                    if ev.mask & libc::IN_MOVE_SELF != 0 {
                        // SAFETY: removing our own watch.
                        unsafe { libc::inotify_rm_watch(self.fd.as_raw_fd(), ev.wd) };
                    }
                    self.dir_wd = None;
                    relevant = true;
                } else if files.iter().any(|f| f.as_bytes() == name) {
                    relevant = true;
                }
            } else if Some(ev.wd) == self.parent_wd && name == self.dir_name.as_slice() {
                // The folder appeared (or another took its place): watch that one.
                self.dir_wd = add_watch(&self.fd, &self.dir, FILE_EVENTS).ok();
                relevant = true;
            }
        }
        Ok(relevant)
    }
}

fn add_watch(fd: &OwnedFd, path: &Path, mask: u32) -> io::Result<i32> {
    let c = CString::new(path.as_os_str().as_bytes())
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
    // SAFETY: `c` is a valid NUL-terminated path for the duration of the call.
    let wd = unsafe { libc::inotify_add_watch(fd.as_raw_fd(), c.as_ptr(), mask) };
    if wd < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(wd)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn wait_for(w: &BindingsWatch) -> bool {
        let until = Instant::now() + Duration::from_secs(2);
        while Instant::now() < until {
            if w.take() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        false
    }

    fn quiet(w: &BindingsWatch) -> bool {
        std::thread::sleep(SETTLE * 3);
        !w.take()
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn sees_real_changes_only() {
        let root = std::env::temp_dir().join(format!("xrizer-watch-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        let dir = root.join("xrizer");
        let w = BindingsWatch::start(dir.clone(), vec!["knuckles.json".into()]).unwrap();
        std::thread::sleep(SETTLE);

        // The folder doesn't exist yet: its first binding is seen.
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("knuckles.json"), "{}").unwrap();
        assert!(wait_for(&w), "a new binding");

        // The same bytes again, or another controller's file: nothing to do.
        std::fs::write(dir.join("knuckles.json"), "{}").unwrap();
        assert!(quiet(&w), "an identical save");
        std::fs::write(dir.join("oculustouch.json"), "{}").unwrap();
        assert!(quiet(&w), "a file it doesn't read");

        // An edit, then one saved through a temp file.
        std::fs::write(dir.join("knuckles.json"), "{\"a\":1}").unwrap();
        assert!(wait_for(&w), "an edit");
        std::fs::write(dir.join(".tmp"), "{\"a\":2}").unwrap();
        std::fs::rename(dir.join(".tmp"), dir.join("knuckles.json")).unwrap();
        assert!(wait_for(&w), "a save through a rename");

        // Reset (moved aside), then the whole folder gone and back.
        std::fs::rename(dir.join("knuckles.json"), dir.join("knuckles.json.bak")).unwrap();
        assert!(wait_for(&w), "a removal");
        std::fs::remove_dir_all(&dir).unwrap();
        assert!(quiet(&w), "nothing was there any more");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("knuckles.json"), "{}").unwrap();
        assert!(wait_for(&w), "a folder made again");

        drop(w);
        let _ = std::fs::remove_dir_all(&root);
    }
}
