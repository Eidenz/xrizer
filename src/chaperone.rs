use crate::openxr_data::RealOpenXrData;
use log::warn;
use openvr as vr;
use openxr as xr;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// How long a play area read from the runtime is reused: some games ask
/// every frame, and each ask is a round trip to the runtime.
const PLAY_AREA_TTL: Duration = Duration::from_secs(1);

/// A play area (width along X, depth along Z), and when it was asked for.
type CachedPlayArea = Option<(Instant, Option<(f32, f32)>)>;

#[derive(macros::InterfaceImpl)]
#[interface = "IVRChaperone"]
#[versions(004, 003)]
pub struct Chaperone {
    vtables: Vtables,
    openxr: Arc<RealOpenXrData>,
    /// The last play area asked for, and when.
    play_area: Mutex<CachedPlayArea>,
}

impl Chaperone {
    pub fn new(openxr: Arc<RealOpenXrData>) -> Self {
        Self {
            vtables: Default::default(),
            openxr,
            play_area: Mutex::new(None),
        }
    }

    /// The play area as the runtime knows it: the stage's bounds (width along
    /// X, depth along Z), centred on the standing origin like OpenVR's.
    fn play_area(&self) -> Option<(f32, f32)> {
        let mut cached = self.play_area.lock().unwrap();
        if let Some((at, area)) = *cached
            && at.elapsed() < PLAY_AREA_TTL
        {
            return area;
        }
        let area = match self
            .openxr
            .session_data
            .get()
            .session
            .reference_space_bounds_rect(xr::ReferenceSpaceType::STAGE)
        {
            Ok(bounds) => bounds
                .map(|b| (b.width, b.height))
                .filter(|(x, z)| *x > 0.0 && *z > 0.0),
            Err(e) => {
                warn!("Couldn't get the play area from the runtime: {e}");
                None
            }
        };
        *cached = Some((Instant::now(), area));
        area
    }
}

/// A play area's corners on the floor, counter-clockwise seen from above.
fn play_area_corners((x, z): (f32, f32)) -> vr::HmdQuad_t {
    let (x, z) = (x / 2.0, z / 2.0);
    let corner = |x, z| vr::HmdVector3_t { v: [x, 0.0, z] };
    vr::HmdQuad_t {
        vCorners: [corner(-x, -z), corner(-x, z), corner(x, z), corner(x, -z)],
    }
}

impl vr::IVRChaperone004_Interface for Chaperone {
    fn ResetZeroPose(&self, origin: vr::ETrackingUniverseOrigin) {
        self.openxr.reset_tracking_space(origin);
    }

    fn ForceBoundsVisible(&self, _: bool) {
        crate::warn_unimplemented!("ForceBoundsVisible");
    }
    fn AreBoundsVisible(&self) -> bool {
        crate::warn_unimplemented!("AreBoundsVisible");
        false
    }
    fn GetBoundsColor(
        &self,
        color_array: *mut vr::HmdColor_t,
        count: std::ffi::c_int,
        _collision_bounds_fade_distance: f32,
        camera_color: *mut vr::HmdColor_t,
    ) {
        crate::warn_unimplemented!("GetBoundsColor");
        if color_array.is_null() || camera_color.is_null() || count <= 0 {
            return;
        }
        let color_array = unsafe { std::slice::from_raw_parts_mut(color_array, count as usize) };
        color_array.fill(vr::HmdColor_t::default());
        unsafe {
            camera_color.write(vr::HmdColor_t::default());
        }
    }
    fn SetSceneColor(&self, _: vr::HmdColor_t) {
        crate::warn_unimplemented!("SetSceneColor");
    }
    fn ReloadInfo(&self) {
        *self.play_area.lock().unwrap() = None;
    }
    fn GetPlayAreaRect(&self, rect: *mut vr::HmdQuad_t) -> bool {
        if rect.is_null() {
            return false;
        }
        let area = self.play_area();
        unsafe {
            *rect = area.map(play_area_corners).unwrap_or_default();
        }
        area.is_some()
    }
    fn GetPlayAreaSize(&self, size_x: *mut f32, size_z: *mut f32) -> bool {
        // Without bounds from the runtime, a token square metre: games expect
        // a size.
        let (x, z) = self.play_area().unwrap_or((1.0, 1.0));
        unsafe {
            if !size_x.is_null() {
                *size_x = x;
            }
            if !size_z.is_null() {
                *size_z = z;
            }
        };
        true
    }
    fn GetCalibrationState(&self) -> vr::ChaperoneCalibrationState {
        vr::ChaperoneCalibrationState::OK
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{clientcore::Injector, openxr_data::OpenXrData};
    use vr::IVRChaperone004_Interface;

    fn chaperone() -> (Arc<RealOpenXrData>, Chaperone) {
        let xr = Arc::new(OpenXrData::new(&Injector::default()).unwrap());
        let chaperone = Chaperone::new(xr.clone());
        (xr, chaperone)
    }

    fn size(c: &Chaperone) -> (f32, f32) {
        let (mut x, mut z) = (0.0, 0.0);
        assert!(c.GetPlayAreaSize(&mut x, &mut z));
        (x, z)
    }

    #[test]
    fn play_area_from_the_stage_bounds() {
        let (xr, c) = chaperone();
        let session = xr.session_data.get().session.as_raw();
        fakexr::set_stage_bounds(
            session,
            Some(xr::Extent2Df {
                width: 3.2,
                height: 2.4,
            }),
        );
        assert_eq!(size(&c), (3.2, 2.4));
        let mut rect = vr::HmdQuad_t::default();
        assert!(c.GetPlayAreaRect(&mut rect));
        let corners = rect.vCorners.map(|c| c.v);
        assert_eq!(
            corners,
            [
                [-1.6, 0.0, -1.2],
                [-1.6, 0.0, 1.2],
                [1.6, 0.0, 1.2],
                [1.6, 0.0, -1.2]
            ]
        );
        // Counter-clockwise seen from above: the outline's normal points up.
        let edge = |i: usize| {
            let (a, b) = (corners[i], corners[(i + 1) % 4]);
            [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
        };
        for i in 0..4 {
            let (u, v) = (edge(i), edge((i + 1) % 4));
            assert!(u[2] * v[0] - u[0] * v[2] > 0.0, "turn {i}");
        }
    }

    #[test]
    fn no_bounds_keeps_the_token_square_metre() {
        let (xr, c) = chaperone();
        fakexr::set_stage_bounds(xr.session_data.get().session.as_raw(), None);
        assert_eq!(size(&c), (1.0, 1.0));
        let mut rect = vr::HmdQuad_t::default();
        assert!(!c.GetPlayAreaRect(&mut rect));
    }

    #[test]
    fn bounds_are_asked_again_after_reload_info() {
        let (xr, c) = chaperone();
        let session = xr.session_data.get().session.as_raw();
        fakexr::set_stage_bounds(session, None);
        assert_eq!(size(&c), (1.0, 1.0));
        fakexr::set_stage_bounds(
            session,
            Some(xr::Extent2Df {
                width: 2.0,
                height: 1.5,
            }),
        );
        // Still the cached answer...
        assert_eq!(size(&c), (1.0, 1.0));
        // ...until the app asks for a reload.
        c.ReloadInfo();
        assert_eq!(size(&c), (2.0, 1.5));
    }
}
